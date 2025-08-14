$fn = 100;

cap_thichkness = 1.5;
base_thickness = 3;
pcb_thickness = 1.6;

difference() {
    union() {
        /*difference() {
            translate([0, 0, cap_thichkness])cylinder(h = 5, d = 48.0);
            translate([0, 0, cap_thichkness+2.5])cylinder(h = 5, d = 42.5);
        }
        cube([cap_thichkness, 51.8, 38.4]);
        cylinder(h = cap_thichkness, d = 50);*/
        //translate([1.6, -(51.8/2), 0])rotate([0, 270, 0])cube([cap_thichkness, 51.8, 38.4]);
        translate([1.6, -(51.8/2), 0])rotate([0, 270, 0])roundedcube([cap_thichkness+0.6, 51.8, 38.4], false, 1.0, "x");
        
        translate([-3.8, -(28/2), -4.5])rotate([0, 270, 0])roundedcube([cap_thichkness+0.6+3, 28, 33], false, 2.5, "x");
        
        //rotate([0, 0, 180])translate([20.5+12, -9.5, 0])cylinder(h = 6, d = 6.5);
        //rotate([0, 0, 180])translate([20.5-12, 9.5, 0])cylinder(h = 6, d = 6.5);
        
        
        
        //rotate([0, 0, 180])translate([0, -17, 1])cube([base_thickness, 34, 110]);
        //rotate([0, 0, 180])translate([base_thickness, -17, 1+112])cube([pcb_thickness, 34, 1]);
        
        

    }
    
    //rotate([0, 0, 180])translate([base_thickness, -25, cap_thichkness+2.5])cube([50, 50, 10]);
    translate([4, -30, cap_thichkness])cube([3.5, 60, 8]);
    //translate([0, -40/2, cap_thichkness])cube([7.5, 40, 8]);
    
    //rj45
    rotate([0, 0, 180])translate([20.5, 0, -10])cylinder(h = 20, d = 24);
    
    rotate([0, 0, 180])translate([20.5+12, -9.5, -10])cylinder(h = 20, d = 3.8);
    rotate([0, 0, 180])translate([20.5-12, 9.5, -10])cylinder(h = 20, d = 3.8);
    
    //panel holes 31 x 45
    translate([-7.4/2+1.6, -45/2, 0])rotate([0, 0, 0])cylinder(h = 5, d = 3.5);
    translate([-7.4/2+1.6, 45/2, 0])rotate([0, 0, 0])cylinder(h = 5, d = 3.5);
    translate([-7.4/2+1.6-31, -45/2, 0])rotate([0, 0, 0])cylinder(h = 5, d = 3.5);
    translate([-7.4/2+1.6-31, 45/2, 0])rotate([0, 0, 0])cylinder(h = 5, d = 3.5);
    
    translate([-7.4/2+1.6, -45/2, 0])rotate([0, 0, 0])cylinder(h = 1.8, d1 = 5, d2 = 3.5);
    translate([-7.4/2+1.6, 45/2, 0])rotate([0, 0, 0])cylinder(h = 1.8, d1 = 5, d2 = 3.5);
    translate([-7.4/2+1.6-31, -45/2, 0])rotate([0, 0, 0])cylinder(h = 1.8, d1 = 5, d2 = 3.5);
    translate([-7.4/2+1.6-31, 45/2, 0])rotate([0, 0, 0])cylinder(h = 1.8, d1 = 5, d2 = 3.5);
    
    



}

//translate([2.5, 7, cap_thichkness])cube([7.5, 11, 2.5]);
//translate([2.5, -7-11, cap_thichkness])cube([7.5, 11, 2.5]);

$fs = 0.15;

module roundedcube(size = [1, 1, 1], center = false, radius = 0.5, apply_to = "all") {
	// If single value, convert to [x, y, z] vector
	size = (size[0] == undef) ? [size, size, size] : size;

	translate_min = radius;
	translate_xmax = size[0] - radius;
	translate_ymax = size[1] - radius;
	translate_zmax = size[2] - radius;

	diameter = radius * 2;

	obj_translate = (center == false) ?
		[0, 0, 0] : [
			-(size[0] / 2),
			-(size[1] / 2),
			-(size[2] / 2)
		];

	translate(v = obj_translate) {
		hull() {
			for (translate_x = [translate_min, translate_xmax]) {
				x_at = (translate_x == translate_min) ? "min" : "max";
				for (translate_y = [translate_min, translate_ymax]) {
					y_at = (translate_y == translate_min) ? "min" : "max";
					for (translate_z = [translate_min, translate_zmax]) {
						z_at = (translate_z == translate_min) ? "min" : "max";

						translate(v = [translate_x, translate_y, translate_z])
						if (
							(apply_to == "all") ||
							(apply_to == "xmin" && x_at == "min") || (apply_to == "xmax" && x_at == "max") ||
							(apply_to == "ymin" && y_at == "min") || (apply_to == "ymax" && y_at == "max") ||
							(apply_to == "zmin" && z_at == "min") || (apply_to == "zmax" && z_at == "max")
						) {
							sphere(r = radius);
						} else {
							rotate = 
								(apply_to == "xmin" || apply_to == "xmax" || apply_to == "x") ? [0, 90, 0] : (
								(apply_to == "ymin" || apply_to == "ymax" || apply_to == "y") ? [90, 90, 0] :
								[0, 0, 0]
							);
							rotate(a = rotate)
							cylinder(h = diameter, r = radius, center = true);
						}
					}
				}
			}
		}
	}
}
