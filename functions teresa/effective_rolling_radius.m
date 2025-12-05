function R_eff = effective_rolling_radius(FZ, tyre_pressure)
%% From Tyre property file
R_0 = 0.2032; %Free tyre radius
R_rim= 0.127; %Nominal rim radius
Cz0 = 200000.0; %Tyre vertical stiffness
BREFF = 8.0; %Low load stiffness effective rolling radius
DREFF = 0.24; %Peak value of effective rolling radius
FREFF = 0.01; %High load stiffness effective rolling radius
Fz0 = 1000.0; %Nominal wheel load
P0= 0.82737; %Reference (nominal) pressure
DPI = (tyre_pressure-P0)/P0;

%% Effective rolling radius
Cz= Cz0*(1+DPI);
R_eff= R_0-Fz0/Cz*(DREFF*atan(BREFF*FZ/Fz0)+FREFF*(FZ/Fz0));