clear;
clc;

fl_suspension = load("car\car_variables\fl_suspension.mat").fl_suspension;

fl_suspension.set_damper_distance(205);

fprintf("Kingpin on ground: "); fl_suspension.get_kingpin_in_ground().print()
fprintf("Contact patch:     "); fl_suspension.get_contact_patch().print()
fprintf("Scrub Radius: %8.2f\n", fl_suspension.get_scrub_radius());
fprintf("Scrub Radius on X: %8.2f\n", abs(fl_suspension.get_kingpin_in_ground().y - fl_suspension.get_contact_patch().y));

fl_suspension.alone_plot3d();

fl_suspension.wheel_print();