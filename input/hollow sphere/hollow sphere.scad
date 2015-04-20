difference() {
    sphere(r=15);
    sphere(r=13);
    translate([0,0,13])
        cube(size = [25,25,8], center = true);
}
