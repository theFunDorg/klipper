// Flatline kinematics stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

struct flatline_stepper {
    struct stepper_kinematics sk;
    double arm2, rail_offset_x, rail_offset_y;
};


// Let's assume that for a given time, we're not calculating the position of a given stepper and then sending it to the motor. 
//We're instead calculating a position for the stepper for a given time, then adding it to the list.
// This means we are creating things to add to the end of a queue that are then executed in essential realtime. So no need to worry about issues with 
static double
flatline_stepper_calc_position(struct stepper_kinematics *sk, struct move *m, double move_time)
{
    // Some sorta pointer fuckery, taking the stepper kinematics and making a subset of it somehow
    struct flatline_stepper *ds = container_of(sk, struct flatline_stepper, sk);
    struct coord c = move_get_coord(m, move_time); // Well this line appears to get the coordinates of the head in xyz at a given time. Fuck. Yeah. 
    double dx = ds->rail_offset_x - c.x;
    return sqrt( ds->arm2 - c.z*c.z - dx*dx ) + c.y - ds->rail_offset_y; // Really need to think about this, does it need to be absolute...? wtf??
};

// b(-+)               a(++)
// |                   |
// |                   |
// |                   |
// |                   |
// |                   |
// _____________________
// 
//   |               |
//   |               |
//   |               |
//   |               |
// _____________________
// 
// |                   |
// |                   |
// |                   |
// |                   |
// |                   |
// c(--)               d(+-)

// Two triangles, one in the xy plane, with formulae:

// dxy^2 = (rox-x)^2 + (roy+g-y)^2, where:
//    rox/y = offsets of rails, 
//    g=stepper position relative to innermost end of rail, 
//    dxy= vector in xy plane of delta from the effector edge to the rail carriage, accounting for all the offsets etc.

// Larm^2 = dxy^2 + (z+roz)^2, where:
//    Larm = length of the delta rods from ball to ball
//    dxy = same as above
//    z = z position of the effector/z extruder tip
//    roz = z offset for the effector, ie distance from horizontal plane of rail carriage-ended delta rod balls to the horizontal plane of the tip of the hotend. 

// g = y - roy + sqrt( ( Larm )^2 - ( z-roz )^2 - ( rox - x )^2 )

// Fuck it drop the z offset, we don't need it. I think it's calculated by the software somehow and passed to the itersolver there. 
// Only thing I can think of going wrong is the section (roy+g-y)^2. But, I think by definition roy>y ALWAYS, so we can take that assumption and use it to get the correct signage

struct stepper_kinematics * __visible
flatline_stepper_alloc(double arm2, double rail_offset_x, double rail_offset_y )
{
    struct flatline_stepper *ds = malloc(sizeof(*ds));
    memset(ds, 0, sizeof(*ds));
    ds->arm2 = arm2;
    ds->rail_offset_x = rail_offset_x;
    ds->rail_offset_y = rail_offset_y;
    ds->sk.calc_position_cb = flatline_stepper_calc_position;
    ds->sk.active_flags = AF_X | AF_Y | AF_Z; // I think this is saying that during this move, ALL 3 axes are in motion, AF=active flag
    return &ds->sk;
}
