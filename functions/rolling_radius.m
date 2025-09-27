
function RR = rolling_radius(fz, camber)
    load('B1654run21.mat');

    % Effective Rolling Radius
    %{
Data from TTC Calsplan
Hoosier R25B 18x7.5-10
0.83 bar
%}

    % IA=0, FZ= {250-50}
    FZ_IA0= FZ(1:7669); %in N
    RE_IA0= RE(1:7669)*10; %in mm
    %curveFitter(FZ_IA0, RE_IA0)
    p1= 0.0058;
    p2= 228.61;
    f1= @(x) p1*x + p2;
    %{
Goodness of Fit
           Value 
SSE         4746.9
R-square    0.8581
DFE           7667
Adj R-sq    0.8581
RMSE        0.7868
%}

    % IA=2 FZ= {200-150}
    FZ_IA2= FZ(7670:14061); %in N
    RE_IA2= RE(7670:14061)*10; %in mm
    %curveFitter(FZ_IA2, RE_IA2)
    p3= 0.0052;
    p4= 227.85;
    f2= @(x) p3*x + p4;
    %{
Goodness of Fit
           Value 
SSE         2792.9
R-square    0.8592
DFE           6390
Adj R-sq    0.8592
RMSE        0.6611
%}

    if camber == 0
        RR= f1(fz);
    elseif camber== 2
        RR= f2(fz);
    else
        RR= (f1(fz)+f2(fz))/2;
    end
end

%{
OLD CODE
%0.83 bar, ia=0, fz={200-150}
v1=sum(RE(1278:3833));
c0=v1/2555;

%0.83 bar, ia=2, fz={200-150}
v2=sum(RE(7670:10225));
c2=v2/2555;

RER=(c0 +c2)/2 %in cm
%}
