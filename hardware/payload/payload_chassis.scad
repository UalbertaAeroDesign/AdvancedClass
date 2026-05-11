// payload_chassis.scad — SAE Aero Design 2026 payload rover chassis
//
// To use:
//   1. Install OpenSCAD (free): https://openscad.org/downloads.html
//   2. Open this file in OpenSCAD
//   3. Edit dimensions below if needed
//   4. F5 = preview, F6 = render
//   5. File → Export → Export as STL
//
// All dimensions in millimetres.

// =====================================================
// DIMENSIONS — edit these
// =====================================================
chassis_l = 130;     // length (front-to-back)
chassis_w = 90;      // width (side-to-side)
chassis_h = 50;      // total height
wall      = 2.5;     // wall thickness

// Raspberry Pi 5 mounting (centred in chassis floor)
rpi_hole_x_spacing = 58;
rpi_hole_y_spacing = 49;
rpi_hole_dia       = 2.7;   // M2.5 clearance
standoff_dia       = 5.5;
standoff_height    = 8;

// Wheel axle holes (cut through both side walls)
wheel_axle_dia        = 4;
wheel_axle_from_rear  = 32;
wheel_axle_from_floor = 11;   // = wheel_radius − ground_clearance

// Camera cutout in front wall
cam_cutout_w = 18;
cam_cutout_h = 18;
cam_cutout_z = 28;            // centre height above outer chassis bottom

// Cable pass-throughs in rear wall (×2)
cable_hole_dia      = 8;
cable_hole_z        = 32;
cable_hole_y_offset = 18;     // ±distance from chassis centreline

// Ball caster mount (4-hole pattern in floor near front)
caster_hole_dia       = 2.5;
caster_hole_x_spacing = 14;
caster_hole_y_spacing = 14;
caster_from_front     = 18;

// Rendering quality (higher = smoother circles, slower render)
$fn = 48;


// =====================================================
// MODEL
// =====================================================
// Origin is at the back-left-bottom corner.
// X = forward, Y = right (looking from behind), Z = up.

eps = 0.5;   // small overlap so holes cleanly cut through walls

module chassis() {
    difference() {
        union() {
            // Outer shell
            difference() {
                cube([chassis_l, chassis_w, chassis_h]);
                // Hollow interior (open top)
                translate([wall, wall, wall])
                    cube([chassis_l - 2*wall,
                          chassis_w - 2*wall,
                          chassis_h]);
            }

            // RPi standoffs (4 cylinders rising from the floor)
            for (dx = [-rpi_hole_x_spacing/2, rpi_hole_x_spacing/2])
                for (dy = [-rpi_hole_y_spacing/2, rpi_hole_y_spacing/2])
                    translate([chassis_l/2 + dx,
                               chassis_w/2 + dy,
                               wall])
                        cylinder(h = standoff_height,
                                 d = standoff_dia);
        }

        // ===== HOLES (subtracted from above) =====

        // Wheel axle hole through both side walls
        translate([wheel_axle_from_rear, -eps, wheel_axle_from_floor])
            rotate([-90, 0, 0])
                cylinder(h = chassis_w + 2*eps,
                         d = wheel_axle_dia);

        // Camera cutout in front wall (+X side)
        translate([chassis_l - wall - eps,
                   (chassis_w - cam_cutout_w)/2,
                   cam_cutout_z - cam_cutout_h/2])
            cube([wall + 2*eps, cam_cutout_w, cam_cutout_h]);

        // Cable pass-throughs in rear wall (×2)
        for (offset = [-cable_hole_y_offset, cable_hole_y_offset])
            translate([-eps,
                       chassis_w/2 + offset,
                       cable_hole_z])
                rotate([0, 90, 0])
                    cylinder(h = wall + 2*eps,
                             d = cable_hole_dia);

        // RPi M2.5 holes (through standoffs and floor)
        for (dx = [-rpi_hole_x_spacing/2, rpi_hole_x_spacing/2])
            for (dy = [-rpi_hole_y_spacing/2, rpi_hole_y_spacing/2])
                translate([chassis_l/2 + dx,
                           chassis_w/2 + dy,
                           -eps])
                    cylinder(h = wall + standoff_height + 2*eps,
                             d = rpi_hole_dia);

        // Ball caster mounting holes (through floor)
        caster_x = chassis_l - caster_from_front;
        for (dx = [-caster_hole_x_spacing/2, caster_hole_x_spacing/2])
            for (dy = [-caster_hole_y_spacing/2, caster_hole_y_spacing/2])
                translate([caster_x + dx,
                           chassis_w/2 + dy,
                           -eps])
                    cylinder(h = wall + 2*eps,
                             d = caster_hole_dia);
    }
}

chassis();
