function rack_displacement= rack_displacement_calculator(steering, doble_cardan, steering_wheel_angle_deg, cardan_angle_deg)
    arguments
        steering steering
    end
    
    %Rack displacement for a right turn
    %{
    Cardan status:
    doble_cardan= true, doble cardan or two single cardan (with the same
    cardan_angle_deg
    doble_cardan= false, one single cardan
    %}

    if doble_cardan== true
        rotation_at_rack_rad= steering_wheel_angle_deg*pi/180;
    else
        rotation_at_rack_rad= atan2(sin(steering_wheel_angle_deg*pi/180),cos(cardan_angle_deg*pi/180)*cos(steering_wheel_angle_deg*pi/180));
    end
    
    if steering.front_pinion == true
        rack_displacement= -rotation_at_rack_rad*steering.pinion_diameter;
    else
        rack_displacement = rotation_at_rack_rad * steering.pinion_diameter;
    end
end