# Code for handling the kinematics of linear Flatline robots
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import stepper, mathutil

# Slow moves once the ratio of tower to XY movement exceeds SLOW_RATIO
SLOW_RATIO = 3.

## 
class FlatlineKinematics:
    ## Initialising kinematics
    def __init__(self, toolhead, config):

        # Setup horizontal rails
        stepper_configs = [config.getsection('stepper_' + a) for a in 'abcd']
        rail_a = stepper.LookupMultiRail( stepper_configs[0], need_position_minmax = False)
        a_endstop = rail_a.get_homing_info().position_endstop ## Soooo I think that this just means that it takes the endstop value from the config file, then it gets set for all of the steppers afterwards to this value. Makes sense in a delta, does it in a flat one though?
        rail_b = stepper.LookupMultiRail( stepper_configs[1], need_position_minmax = False, default_position_endstop=a_endstop)
        rail_c = stepper.LookupMultiRail( stepper_configs[2], need_position_minmax = False, default_position_endstop=a_endstop)
        rail_d = stepper.LookupMultiRail( stepper_configs[3], need_position_minmax = False, default_position_endstop=a_endstop)
        self.rails = [rail_a, rail_b, rail_c, rail_d]
        config.get_printer().register_event_handler("stepper_enable:motor_off", self._motor_off)

        # Setup max velocity
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat( 'max_z_velocity', self.max_velocity, above=0., maxval=self.max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', self.max_accel, above=0., maxval=self.max_accel)

        # Read radius and arm lengths
        arm_length_a = stepper_configs[0].getfloat('arm_length', above=0.) ## I guess we just define this once and not for the whole set, assuming they're all identical of course
        self.arm_lengths = arm_lengths = [sconfig.getfloat('arm_length', arm_length_a, above=0) for sconfig in stepper_configs]
        self.arm2 = [arm**2 for arm in arm_lengths] ## square it now and forever hold your piece. Derp



        # Determine tower locations in cartesian space
        #self.angles = [sconfig.getfloat('angle', angle) for sconfig, angle in zip(stepper_configs, [210., 330., 90.])] self.towers = [(math.cos(math.radians(angle)) * radius, math.sin(math.radians(angle)) * radius) for angle in self.angles]
        # Leaving this in for now but I think this is the maths that is used 
        #self.towers = [(math.cos(math.radians(angle)) * radius, math.sin(math.radians(angle)) * radius) for angle in self.angles]
        # Setting x and y offsets. When given x and y, subtract these values and use them to determine the triangles defining motion.
        rail_serial_spacing = config.getfloat('rail_serial_spacing', above=0.)
        rail_parallel_spacing = config.getfloat('rail_parallel_spacing', above=0.)
        effector_serial_offset = config.getfloat('effector_serial_offset', above=0.)
        effector_parallel_offset = config.getfloat('effector_parallel_offset', above=0.)
        rail_offset_a = [ rail_parallel_spacing*0.5 - effector_parallel_offset*0.5,  rail_serial_spacing*0.5 - effector_serial_offset*0.5 ]
        rail_offset_b = [-rail_parallel_spacing*0.5 + effector_parallel_offset*0.5,  rail_serial_spacing*0.5 - effector_serial_offset*0.5 ]
        rail_offset_c = [-rail_parallel_spacing*0.5 + effector_parallel_offset*0.5, -rail_serial_spacing*0.5 + effector_serial_offset*0.5 ]
        rail_offset_d = [ rail_parallel_spacing*0.5 - effector_parallel_offset*0.5, -rail_serial_spacing*0.5 + effector_serial_offset*0.5 ]
        self.rail_offsets = [ rail_offset_a, rail_offset_b, rail_offset_c, rail_offset_d ]

        #maybe change rail_origin to rail_offset_a/b/c/d, where the offset is using the dimensions of the rails AND the effector dimensions

        # All rails will have value of 0 at their closest point to the center of the printer, ie closest to x,y=0,0. So the maths makes sense
        for r, a, ro in zip(self.rails, self.arm2, self.rail_offsets): # this iterates through all 4 rails to resolve positions. 
            r.setup_itersolve('flatline_stepper_alloc', a, ro[0], ro[1])
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)


        # Ooooh, check this below... I honestly am tempted to code this out for now. We don't have the "too fast xy movement" issue with this format
        # Though we maaaaay have it with the x direction as we have 2 sets of motors working against each other. 
        self.abs_endstops = [(rail.get_homing_info().position_endstop + math.sqrt(arm2 - radius**2)) for rail, arm2 in zip(self.rails, self.arm2)]
        # So for above I will need to recalculate all my endstops using something... pythagorean. I guess it would be a right angle triangle of...
        # Okay in my case... I need these endstops on the inside of the rails rather than the outside. 
        # The outsides will never get met as the z axis endstop should prevent that


        # Setup boundary checks
        self.need_home = True
        self.limit_xy2 = -1.
        self.home_position = tuple( self._actuator_to_cartesian(self.abs_endstops))
        self.max_z = min([rail.get_homing_info().position_endstop for rail in self.rails])
        self.min_z = config.getfloat('minimum_z_position', 0, maxval=self.max_z)
        self.limit_z = min([ep - arm for ep, arm in zip(self.abs_endstops, arm_lengths)])
        self.min_arm_length = min_arm_length = min(arm_lengths)
        self.min_arm2 = min_arm_length**2
        logging.info( "Delta max build height %.2fmm (radius tapered above %.2fmm)" % (self.max_z, self.limit_z))

        # Find the point where an XY move could result in excessive
        # tower movement
        half_min_step_dist = min([r.get_steppers()[0].get_step_dist() for r in self.rails]) * .5
        min_arm_length = min(arm_lengths)
        ##     
        def ratio_to_xy(ratio):
            return (ratio * math.sqrt(min_arm_length**2 / (ratio**2 + 1.) - half_min_step_dist**2) + half_min_step_dist - radius)
        
        self.slow_xy2 = ratio_to_xy(SLOW_RATIO)**2
        self.very_slow_xy2 = ratio_to_xy(2. * SLOW_RATIO)**2
        self.max_xy2 = min(print_radius, min_arm_length - radius, ratio_to_xy(4. * SLOW_RATIO))**2
        max_xy = math.sqrt(self.max_xy2)
        logging.info("Delta max build radius %.2fmm (moves slowed past %.2fmm" " and %.2fmm)" % (max_xy, math.sqrt(self.slow_xy2), math.sqrt(self.very_slow_xy2)))
        self.axes_min = toolhead.Coord(-max_xy, -max_xy, self.min_z, 0.)
        self.axes_max = toolhead.Coord(max_xy, max_xy, self.max_z, 0.)
        self.set_position([0., 0., 0.], ())

    ## Seems to be a self referring function but I guess is fine to leave as it is...?
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    ## Seems to convert the given coordinates and for the givem machine into a set of cartesian coordinates, this will probably (not) be very needed in my one
    def _actuator_to_cartesian(self, spos):
        sphere_coords = [(t[0], t[1], sp) for t, sp in zip(self.towers, spos)]
        return mathutil.trilateration(sphere_coords, self.arm2)

    ## calculate the position of the printhead based on the position of the steppers. Part of the above, not sure why they bothered to make two separate functions though
    def calc_position(self, stepper_positions):
        spos = [stepper_positions[rail.get_name()] for rail in self.rails]
        return self._actuator_to_cartesian(spos)

    ## Seems to set a position of the head on inputs and tells it to be homed, not needing home. 
    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
        self.limit_xy2 = -1.
        if tuple(homing_axes) == (0, 1, 2):
            self.need_home = False

    ## Function that homes the axes I guess?
    ## Method to home axes:
    ## Move all up, till an endstop hit
    ## Move all down, till an endstop hit
    ## The two that are not endstopped, move towards build plate till one hits
    ## 
    def home(self, homing_state):
        # All axes are homed simultaneously
        homing_state.set_axes([0, 1, 2, 3])
        forcepos = list(self.home_position)
        forcepos[2] = -1.5 * math.sqrt(max(self.arm2)-self.max_xy2)
        homing_state.home_rails(self.rails, forcepos, self.home_position)

    ## Function that I guess tells the machine that the toolhead is going to need to be homed 
    def _motor_off(self, print_time):
        self.limit_xy2 = -1.
        self.need_home = True

    ## Check if the move that is to be carried out is a valid move
    def check_move(self, move):
        end_pos = move.end_pos
        end_xy2 = end_pos[0]**2 + end_pos[1]**2
        if end_xy2 <= self.limit_xy2 and not move.axes_d[2]:
            # Normal XY move
            return
        if self.need_home:
            raise move.move_error("Must home first")
        end_z = end_pos[2]
        limit_xy2 = self.max_xy2
        ##
        if end_z > self.limit_z:
            above_z_limit = end_z - self.limit_z
            allowed_radius = self.radius - math.sqrt( self.min_arm2 - (self.min_arm_length - above_z_limit)**2 )
            limit_xy2 = min(limit_xy2, allowed_radius**2)
        ##
        if end_xy2 > limit_xy2 or end_z > self.max_z or end_z < self.min_z:
            # Move out of range - verify not a homing move
            if (end_pos[:2] != self.home_position[:2] or end_z < self.min_z or end_z > self.home_position[2]):
                raise move.move_error()
            limit_xy2 = -1.
        ##
        if move.axes_d[2]:
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
            limit_xy2 = -1.
        # Limit the speed/accel of this move if is is at the extreme
        # end of the build envelope
        extreme_xy2 = max(end_xy2, move.start_pos[0]**2 + move.start_pos[1]**2)
        ##
        if extreme_xy2 > self.slow_xy2:
            r = 0.5
            if extreme_xy2 > self.very_slow_xy2:
                r = 0.25
            move.limit_speed(self.max_velocity * r, self.max_accel * r)
            limit_xy2 = -1.
        self.limit_xy2 = min(limit_xy2, self.slow_xy2)

    ## 
    def get_status(self, eventtime):
        return {
            'homed_axes': '' if self.need_home else 'xyz',
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
##            'cone_start_z': self.limit_z,
        }





# ==================================================================================================================
# ==================================================================================================================
# Flatline parameter calibration for DELTA_CALIBRATE tool
# ==================================================================================================================
# ==================================================================================================================





    ## This should be a part of above but fuck it drop it for now...
    def get_calibration(self):
        endstops = [rail.get_homing_info().position_endstop
                    for rail in self.rails]
        stepdists = [rail.get_steppers()[0].get_step_dist()
                     for rail in self.rails]
        return FlatlineCalibration(self.radius, self.angles, self.arm_lengths,
                                endstops, stepdists)

class FlatlineCalibration:

    def __init__(self, radius, angles, arms, endstops, stepdists):
        self.radius = radius
        self.angles = angles
        self.arms = arms
        self.endstops = endstops
        self.stepdists = stepdists
        # Calculate the XY cartesian coordinates of the Flatline rails
        radian_angles = [math.radians(a) for a in angles]
        self.towers = [(math.cos(a) * radius, math.sin(a) * radius)
                       for a in radian_angles]
        # Calculate the absolute Z height of each tower endstop
        radius2 = radius**2
        self.abs_endstops = [e + math.sqrt(a**2 - radius2) 
                             for e, a in zip(endstops, arms)]

    ## 
    def coordinate_descent_params(self, is_extended):
        # Determine adjustment parameters (for use with coordinate_descent)
        adj_params = ('radius', 'angle_a', 'angle_b', 'angle_c',
                      'endstop_a', 'endstop_b', 'endstop_c', 'endstop_d')
        if is_extended:
            adj_params += ('arm_a', 'arm_b', 'arm_c', 'arm_d')
        params = { 'radius': self.radius }
        for i, axis in enumerate('abcd'):
            params['angle_'+axis] = self.angles[i]
            params['arm_'+axis] = self.arms[i]
            params['endstop_'+axis] = self.endstops[i]
            params['stepdist_'+axis] = self.stepdists[i]
        return adj_params, params

    ## 
    def new_calibration(self, params):
        # Create a new calibration object from coordinate_descent params
        radius = params['radius']
        angles = [params['angle_'+a] for a in 'abc']
        arms = [params['arm_'+a] for a in 'abc']
        endstops = [params['endstop_'+a] for a in 'abc']
        stepdists = [params['stepdist_'+a] for a in 'abc']
        return FlatlineCalibration(radius, angles, arms, endstops, stepdists)

    ## 
    def get_position_from_stable(self, stable_position):
        # Return cartesian coordinates for the given stable_position
        sphere_coords = [
            (t[0], t[1], es - sp * sd)
            for sd, t, es, sp in zip(self.stepdists, self.towers,
                                     self.abs_endstops, stable_position) ]
        return mathutil.trilateration(sphere_coords, [a**2 for a in self.arms])

    ## 
    def calc_stable_position(self, coord):
        # Return a stable_position from a cartesian coordinate
        steppos = [
            math.sqrt(a**2 - (t[0]-coord[0])**2 - (t[1]-coord[1])**2) + coord[2]
            for t, a in zip(self.towers, self.arms) ]
        return [(ep - sp) / sd
                for sd, ep, sp in zip(self.stepdists,
                                      self.abs_endstops, steppos)]

    ## 
    def save_state(self, configfile):
        # Save the current parameters (for use with SAVE_CONFIG)
        configfile.set('printer', 'delta_radius', "%.6f" % (self.radius,))
        for i, axis in enumerate('abcd'):
            configfile.set('stepper_'+axis, 'angle', "%.6f" % (self.angles[i],))
            configfile.set('stepper_'+axis, 'arm_length',
                           "%.6f" % (self.arms[i],))
            configfile.set('stepper_'+axis, 'position_endstop',
                           "%.6f" % (self.endstops[i],))
        gcode = configfile.get_printer().lookup_object("gcode")
        gcode.respond_info(
            "stepper_a: position_endstop: %.6f angle: %.6f arm_length: %.6f\n"
            "stepper_b: position_endstop: %.6f angle: %.6f arm_length: %.6f\n"
            "stepper_c: position_endstop: %.6f angle: %.6f arm_length: %.6f\n"
            "stepper_d: position_endstop: %.6f angle: %.6f arm_length: %.6f\n"
            "delta_radius: %.6f"
            % (self.endstops[0], self.angles[0], self.arms[0],
               self.endstops[1], self.angles[1], self.arms[1],
               self.endstops[2], self.angles[2], self.arms[2],
               self.endstops[3], self.angles[3], self.arms[3],
               self.radius))





# ==================================================================================================================
# ==================================================================================================================
# Flatline parameter calibration for DELTA_CALIBRATE tool
# ==================================================================================================================
# ==================================================================================================================






## Final bitty to load the kinematics
def load_kinematics(toolhead, config):
    return FlatlineKinematics(toolhead, config)
