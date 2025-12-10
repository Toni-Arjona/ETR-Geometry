
function [fz_FR, fz_FL, fz_RR, fz_RL]= normal_load_per_tire_complete(car_weight, ay, trackwidth_front, trackwidth_rear, rollcenter_front, ...
    rollcenter_rear, rollstiffness_front, rollstiffness_rear, H, wheelbase, a, v)
    
    df_coeff = 4.18; % coeficiente downforce
    dr_coeff = 1.3; % coeficiente drag
    air_d = 1.225; % densidad del aire [kg/m^3]
    area = 0.563; % área frontal del coche [m^2]

    g=9.81; %Gravity in m/s^2
    dForce = 0.5*air_d*area*df_coeff*v^2; %dforce a velocidad dada
   
    % H is the distance between the total car's CG and the NRA.
    % Neutral Roll Axis (NRA): The axis joining the front and rear
    % rollcenters
    % a is the distance between the front wheels and the car's CG
    % This calculations are for a right turning car
     
     WTF=ay*((g*car_weight/trackwidth_front)*((H*rollstiffness_front/(rollstiffness_front+rollstiffness_rear))+((wheelbase-a)*rollcenter_front/wheelbase))); %weight transfer front (ay in g's)
     WTR=ay*((g*car_weight/trackwidth_rear)*((H*rollstiffness_rear/(rollstiffness_front+rollstiffness_rear))+(a*rollcenter_rear/wheelbase))); %weight transfer rear (ay in g's)
            
     WTFL=WTF;
     WTFR=-WTF;
     WTRL=WTR;
     WTRR=-WTR;

     wf= car_weight*g*((wheelbase-a)/wheelbase);
     wr= car_weight*g*(a/wheelbase);

     fz_FR= wf/2+ WTFR + dForce/4;
     fz_FL= wf/2+ WTFL + dForce/4;
     fz_RR= wf/2+ WTRR + dForce/4;
     fz_RL= wf/2+ WTRL + dForce/4;
    
end


     