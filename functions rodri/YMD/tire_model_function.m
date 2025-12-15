%% FUNCIÓN MODELO NEUMÁTICO PARA YMD
function [Fy, Fx, Mz] = tire_model(slip_angle, CG_slip, steer, z_load)

lambdaHy=0;
lambdaVy=1;
lambdauy=0.805; %scaling factor
lambdaKygamma=1;
lambdaCy=1.22;
lambdaEy=1;
lambdaKyalpha=1;
lambdaKygamma=1.07;
PF=0.95; %Front pressure Bar
PR=0.95; %Rrear pressure Bar
gammaF0=-2; %Front Camber deg en negatiu
gammaR0=-1.5; %Rear Camber deg en negatiu

%Propietats del neumàtic
PCY1 = 1.477553032924075; %shape factor (Afilar cantonades i estirar i atxatar)
PDY1 = 2.419662634889429; %estirar o aixafar gràfica
PDY2 =- 0.096991237613422;
PDY3 = 9.699367924763585;
PEY1 = - 0.001014423462658; %afilar cantonades
PEY2 = - 0.001567587511733;
PEY3 = -26.252507850904447; %en negatiu s'afila la dreta i en postiu l'esquerra
PEY4 = 4.418491888020646e+03;
PEY5 = 0;%No esta
PKY1 = - 48.096691177591670;
PKY2 = 1.79989978802; %rotar sobre el centre antihorari
PKY3 = 0.776633692944650; 
PKY4 = 1; 
PKY5 = 1; 
PKY6 = 1;
PKY7 = 1;
PHY1 = 9.632469743033963e-05; %Desplaçar lateralment
PHY2 = 4.485102635543676e-04; %Desplaçar lateralment
PVY1 = - 0.005996641445337; %pujar i baixar la grafica sencera
PVY2 = 0.031420448279870; %pujar i baixar la grafica sencera
PVY3 = - 0.840635268453471; %pujar i baixar la grafica sencera
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

MZ=@(SA)  a0m + a1m*cos(SA*wm) + b1m*sin(SA*wm) + a2m*cos(2*SA*wm) + b2m*sin(2*SA*wm) + a3m*cos(3*SA*wm) + b3m*sin(3*SA*wm) + a4m*cos(4*SA*wm) + b4m*sin(4*SA*wm) + a5m*cos(5*SA*wm) + b5m*sin(5*SA*wm);

