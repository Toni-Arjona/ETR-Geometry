%% FUNCIÓN MODELO NEUMÁTICO PARA YMD
function [Fy, Mz] = tire_model_function(slip, camber, Fz)

lambdaHy=0;
lambdaVy=1;
lambdauy=0.805; 
lambdaKygamma=1;
lambdaCy=1.22;
lambdaEy=1;
lambdaKyalpha=1;
lambdaKygamma=1.07;

% Setup tires
PF=0.827; %Front pressure Bar
PR=0.827; %Rrear pressure Bar

%Propietats del neumàtic
PCY1 = 1.477553032924075; 
PDY1 = 2.419662634889429; 
PDY2 =- 0.096991237613422;
PDY3 = 9.699367924763585;
PEY1 = - 0.001014423462658; 
PEY2 = - 0.001567587511733;
PEY3 = -26.252507850904447; 
PEY4 = 4.418491888020646e+03;
PEY5 = 0;
PKY1 = - 48.096691177591670;
PKY2 = 1.79989978802; 
PKY3 = 0.776633692944650; 
PKY4 = 1; 
PKY5 = 1; 
PKY6 = 1;
PKY7 = 1;
PHY1 = 9.632469743033963e-05; 
PHY2 = 4.485102635543676e-04; 
PVY1 = - 0.005996641445337; 
PVY2 = 0.031420448279870; 
PVY3 = - 0.840635268453471; 
PVY4 = - 0.482847397434584;
PPY1 = 0.006522910697878; 
PPY2 = 0.732182121539935;
PPY3 = - 0.248004155399173;
PPY4 =  0.344037334268957;
PPY5 = 0;
Fz0 = 1000;
P0=0.82737;

%Turn slip
z0=1;
z1=1;
z2=1;
z3=1;
z4=1;

%Ecuacio MZ(SA) Regressio 
a0m =       2.107;
a1m =      -3.933;
b1m =       61.61;
a2m =     -0.4837;
b2m =       10.06;
a3m =      0.7475;
b3m =        7.31;
a4m =      0.4277;
b4m =      -0.423;
a5m =       1.124;
b5m =       3.801;
wm =      0.2417;

MZz =@(SA)  a0m + a1m*cos(SA*wm) + b1m*sin(SA*wm) + a2m*cos(2*SA*wm) + b2m*sin(2*SA*wm) + a3m*cos(3*SA*wm) + b3m*sin(3*SA*wm) + a4m*cos(4*SA*wm) + b4m*sin(4*SA*wm) + a5m*cos(5*SA*wm) + b5m*sin(5*SA*wm);


%% MAGIC FORMULA
dfz = (Fz - Fz0)./Fz0;
dpi = (PF - P0)/P0;
Shy0 = (PHY1 + PHY2*dfz)*lambdaHy;
Svy0 = Fz*(PVY1 + PVY2*dfz)*lambdaVy*lambdauy*z2;
Svygamma = Fz*(PVY3 + PVY4*dfz)*camber*lambdaKygamma*lambdauy*z2;
Svy = Svy0 + Svygamma;
Kygamma0 = (PKY6 + PKY7*dfz)*Fz*lambdaKygamma*(1 + PPY5*dpi);
Kyalpha0 = PKY1*Fz0*(1 + PPY1*dpi)*sin(PKY4*atan(Fz/((PKY2*Fz0*(1 + PPY2*dpi)))))*lambdaKyalpha;
Kyalpha = PKY1*Fz0*(1 + PPY1*dpi)*sin(PKY4*atan(Fz/((PKY2 + PKY5*camber^2)*Fz0*(1 + PPY2*dpi))))*(1 - PKY3*abs(camber))*lambdaKyalpha*z3;
Shygamma = ((Kygamma0*camber - Svygamma)/Kyalpha)*z0 + z4 - 1;
Shy = Shy0 + Shygamma;
alphay = -(slip + Shy);
Cy = PCY1*lambdaCy;
uy = (PDY1 + PDY2*dfz)*(1 + PPY3*dpi + PPY4*dpi^2)*(1 - PDY3*camber^2)*lambdauy;
Dy = uy*Fz*z2;
Ey = (PEY1 + PEY2*dfz)*(1 + PEY5*camber^2 - (PEY3 + PEY4*camber)*sign(alphay))*lambdaEy;
By = Kyalpha/(Cy*Dy);



Fy = Dy*sin(Cy*atan(By*alphay - Ey.*(By*alphay - atan(By*alphay)))) + Svy;

Mz = MZz(slip);

            