function [fz_FR, fz_FL, fz_RR, fz_RL]= normal_load_per_tire_simple(car_weight, ay, trackwidth_front, trackwidth_rear, wheelbase,height_CG, a)
    g=9.81; %gravity en m/s^2
    % a is the distance between the front wheels and the car's CG
    % This calculations are for a right turning car
    wf= car_weight*g*(a-wheelbase)/wheelbase;
    wr= car_weight*g*(a)/wheelbase;
    WTF=car_weight*g*ay*height_CG/trackwidth_front;
    WTF=car_weight*g*ay*height_CG/trackwidth_rear;
    
     WTFL=WTF;
     WTFR=-WTF;
     WTRL=WTR;
     WTRR=-WTR;

     fz_FR= wf/2+ WTFR;
     fz_FL= wf/2+ WTFL;
     fz_RR= wf/2+ WTRR;
     fz_RL= wf/2+ WTRL;
end