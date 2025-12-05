function Long_force_coef = longitudinal_force(m, r_eff, camber_deg, slip_ratio, velocity, pressure)
g=9.81; % m/s^2
W = m*g; % vehicle weight in N
h = 0.3; % cog height m
l = 1.6; % wheel base m
cogx= 0.8; % Front axle -> Mass center distance in m
R = r_eff;
CAMdeg = camber_deg; %inclination angle en graus
CAM = CAMdeg*pi/180; %Camber in radians

%Longitudinal Coefficients
PCX1= 1.56182385630461;
PDX1=2.85110165444411;
PDX2=- 0.407371062621199;
PPX1=- 0.705319191164713;
PPX2=- 0.589490385312056;
PPX3=- 0.242820291164767;
PPX4=0.205179565555030;
PDX3=19.2614164371048;
PEX1=0.422733521935518;
PEX2=0.0201014651759272;
PEX3=0.00229620366835657;
PEX4=0.00999560321822648 ;
PKX1=52.2555801076937;
PKX2=- 0.00614387636030559;
PKX3=- 0.155693449933703;
PHX1=0.000340072159079639;
PHX2=- 0.00161506337261505;
%PDXP1=
%PDXP2=
%PDXP3=
%RHX1=
%REX1=
%REX2=
%RCX1=
%RBX1=
%RBX2=
%RBX3=
SVX=1;

%Scaling coefficients
LCX=0.995; %Scale factor of Fx shape factor; valor inventat
LMUX=0.78; %Scale factor of Fx peak friction coefficient; valor inventat
LEX=1; %Scale factor of Fx curvature factor; valor inventat
LKXk=1; %Scale factor of slip stiffness; valor inventat 
LHX=1; %Scale factor of Fx horizontal shift; valor inventat
LXA=1; %Scale factor of alpha influence on Fx; valor inventat

%Data
VV = (0:5:90)/3.6; %Vehicle Velocity m/s
PI = pressure; %pressio en bar
P0= 0.82737;% reference (nominal) tyre pressure
DPI = (PI-P0)/P0;
        
Acx  = 0; %Acceleracio inicial en m/s
fzr  = W*cogx/l; % Rear axle load load
fzf  = W-fzr ; % Front axle load load

for k=1:100
    V  = velocity*slip_ratio; % Wheel velocity m/s
    k  = (V -velocity)/velocity; %slip(i)
    WTx  = (h/l)*m*Acx ; %Longitudinal weight transfer N
    WTF  = -WTx ; 
    WTR  = WTx ;            
    FZf  = fzf  + WTF ;% Front axle load + weight transfer
    FZr  = fzr  + WTR ;% Rear axle load + weight transfer
    FZFL  = FZf /2; % Front left wheel load
    FZFR  = FZf /2; % Front right wheel load
    FZRL  = FZr /2; % Rear left wheel load
    FZRR  = FZr /2; % Rear right wheel load
    DFZFL  = (FZFL -1000)/1000; %load diferential FL
    DFZFR  = (FZFR -1000)/1000; %load diferential FR
    DFZRL  = (FZRL -1000)/1000; %load diferential RL
    DFZRR  = (FZRR -1000)/1000; %load diferential RR
    %FRONT LEFT
    %MF-Tyre 5.2, 6.0, 6.1
    s1=1;
    SHXFL = (PHX1+PHX2*DFZFL )*LHX; 

    %Pure Slip
    kxFL  = k +SHXFL ;
    CXFL  = PCX1*LCX;
    UFL   = (PDX1+PDX2*DFZFL )*(1+PPX3*DPI+PPX4*DPI^2)*(1-PDX3*CAM^2)*LMUX;
    DXFL  = UFL *FZFL *s1;
    ExFL  = (PEX1+PEX2*DFZFL +PEX3*DFZFL ^2)*(1-PEX4*sign(kxFL ))*LEX;
    KXkFL  = FZFL *(PKX1+PKX2*DFZFL )*exp(PKX3*DFZFL )*(1+PPX1*DPI+PPX2*DPI^2)*LKXk; 
    BXFL  = KXkFL /(CXFL *DXFL );
    %Combined slip
    %{
            SHalfaFL= RHX1
            alfaSFL= alfaF+SHalfaFL
            BXalfaFL= (RBX1+RBX3*CAM^2)*cos(arctan(RBX2*k))*LXA
            CXalfaFL= RCX1
            EXAFL= REX1+REX2*DFZ
            %}
    GxalfaFL=1;
    %{
            GxalfaFL= (cos(CXalfaFL*arctan(BXalfaFL*alfaSFL-EXAFL(BXalfaFL*alfaSFL-arctan(BXalfaFL*alfaSFL)))))/(cos(CXalfaFL*arctan(BXalfaFL*SHalfaFL-EXalfa(BXalfaFL-SHalfaFL-arctan(BXalfaFL*SHXalfaFL)))))
            when combined slip is not used: Gxa=1
            %}
    %Turn slip
    s1=1; 
    %{
    BXP1FL= PDXP1(1+PDXP2*DFZ)*cos(arctan(PDXP3*k))
    s1= cos(arctan(BXP1FL*R*PF))
    %when turn slip not used: s1=1
    %}      
   FxFL =(DXFL *sin(CXFL *atan(BXFL *kxFL -ExFL *(BXFL *kxFL -atan(BXFL *kxFL ))))+SVX)*GxalfaFL;
   %FRONT RIGHT
   %MF-Tyre 5.2, 6.0, 6.1
   s1=1;
   SHXFR = (PHX1+PHX2*DFZFR )*LHX; 
   %Pure Slip
   kxFR  = k +SHXFR ;
   CXFR  = PCX1*LCX;
   UFR   = (PDX1+PDX2*DFZFR )*(1+PPX3*DPI+PPX4*DPI^2)*(1-PDX3*CAM^2)*LMUX;
   DXFR  = UFR *FZFR *s1;
   ExFR  = (PEX1+PEX2*DFZFR +PEX3*DFZFR ^2)*(1-PEX4*sign(kxFR ))*LEX;
   KXkFR  = FZFR *(PKX1+PKX2*DFZFR )*exp(PKX3*DFZFR )*(1+PPX1*DPI+PPX2*DPI^2)*LKXk; 
   BXFR  = KXkFR /(CXFR *DXFR );
   %Combined slip
   %{
   SHalfaFR= RHX1
   alfaSFR= alfaF+SHalfaFR
   BXalfaFR= (RBX1+RBX3*CAM^2)*cos(arctan(RBX2*k))*LXA
   CXalfaFR= RCX1
   EXAFR= REX1+REX2*DFZ
   %}
   GxalfaFR=1;
   %{
            GxalfaFR= (cos(CXalfaFR*arctan(BXalfaFR*alfaSFR-EXAFR(BXalfaFR*alfaSFR-arctan(BXalfaFR*alfaSFR)))))/(cos(CXalfaFR*arctan(BXalfaFR*SHalfaFR-EXalfa(BXalfaFR-SHalfaFR-arctan(BXalfaFR*SHXalfaFR)))))
            when combined slip is not used: Gxa=1
            %}
   %Turn slip
   s1=1; 
   %{
   BXP1FR= PDXP1(1+PDXP2*DFZ)*cos(arctan(PDXP3*k))
   s1= cos(arctan(BXP1FR*R*PF))
   %when turn slip not used: s1=1
    %}
            
    FxFR =(DXFR *sin(CXFR *atan(BXFR *kxFR -ExFR *(BXFR *kxFR -atan(BXFR *kxFR ))))+SVX)*GxalfaFR;
    %REAR LEFT
            
    %MF-Tyre 5.2, 6.0, 6.1
    s1=1;
    SHXRL = (PHX1+PHX2*DFZRL )*LHX; 
            
    %Pure Slip
    kxRL  = k +SHXRL ;
    CXRL  = PCX1*LCX;
    URL   = (PDX1+PDX2*DFZRL )*(1+PPX3*DPI+PPX4*DPI^2)*(1-PDX3*CAM^2)*LMUX;
    DXRL  = URL *FZRL *s1;
    ExRL  = (PEX1+PEX2*DFZRL +PEX3*DFZRL ^2)*(1-PEX4*sign(kxRL ))*LEX;
    KXkRL  = FZRL *(PKX1+PKX2*DFZRL )*exp(PKX3*DFZRL )*(1+PPX1*DPI+PPX2*DPI^2)*LKXk; 
    BXRL  = KXkRL /(CXRL *DXRL );
    %Combined slip
    %{
            SHalfaRL= RHX1
            alfaSRL= alfaF+SHalfaRL
            BXalfaRL= (RBX1+RBX3*CAM^2)*cos(arctan(RBX2*k))*LXA
            CXalfaRL= RCX1
            EXARL= REX1+REX2*DFZ
            %}
    GxalfaRL=1;
    %{
            GxalfaRL= (cos(CXalfaRL*arctan(BXalfaRL*alfaSRL-EXARL(BXalfaRL*alfaSRL-arctan(BXalfaRL*alfaSRL)))))/(cos(CXalfaRL*arctan(BXalfaRL*SHalfaRL-EXalfa(BXalfaRL-SHalfaRL-arctan(BXalfaRL*SHXalfaRL)))))
            when combined slip is not used: Gxa=1
            %}
    %Turn slip
    s1=1; 
    %{
    BXP1RL= PDXP1(1+PDXP2*DFZ)*cos(arctan(PDXP3*k))
    s1= cos(arctan(BXP1RL*R*PF))
    %when turn slip not used: s1=1
    %}
            
    FxRL =(DXRL *sin(CXRL *atan(BXRL *kxRL -ExRL *(BXRL *kxRL -atan(BXRL *kxRL ))))+SVX)*GxalfaRL;
            
    %REAR RIGHT
            
    %MF-Tyre 5.2, 6.0, 6.1
    s1=1;
    SHXRR = (PHX1+PHX2*DFZRR )*LHX; 
            
    %Pure Slip
     kxRR  = k +SHXRR ;
     CXRR  = PCX1*LCX;
     URR   = (PDX1+PDX2*DFZRR )*(1+PPX3*DPI+PPX4*DPI^2)*(1-PDX3*CAM^2)*LMUX;
     DXRR  = URR *FZRR *s1;
     ExRR  = (PEX1+PEX2*DFZRR +PEX3*DFZRR ^2)*(1-PEX4*sign(kxRR ))*LEX;
     KXkRR  = FZRR *(PKX1+PKX2*DFZRR )*exp(PKX3*DFZRR )*(1+PPX1*DPI+PPX2*DPI^2)*LKXk; 
     BXRR  = KXkRR /(CXRR *DXRR );
     %Combined slip
     %{
            SHalfaRR= RHX1
            alfaSRR= alfaF+SHalfaRR
            BXalfaRR= (RBX1+RBX3*CAM^2)*cos(arctan(RBX2*k))*LXA
            CXalfaRR= RCX1
            EXARR= REX1+REX2*DFZ
            %}
     GxalfaRR=1;
     %{
     GxalfaRR= (cos(CXalfaRR*arctan(BXalfaRR*alfaSRR-EXARR(BXalfaRR*alfaSRR-arctan(BXalfaRR*alfaSRR)))))/(cos(CXalfaRR*arctan(BXalfaRR*SHalfaRR-EXalfa(BXalfaRR-SHalfaRR-arctan(BXalfaRR*SHXalfaRR)))))
     when combined slip is not used: Gxa=1
     %}
     %Turn slip
     s1=1; 
     %{
            BXP1RR= PDXP1(1+PDXP2*DFZ)*cos(arctan(PDXP3*k))
            s1= cos(arctan(BXP1RR*R*PF))
            %when turn slip not used: s1=1
            %}
            
     FxRR =(DXRR *sin(CXRR *atan(BXRR *kxRR -ExRR *(BXRR *kxRR -atan(BXRR *kxRR ))))+SVX)*GxalfaRR;
            
     %Total
     Fx4 =FxFL +FxFR +FxRL +FxRR ; %Only 4WD
     Fx2 =FxRL +FxRR ; %2WD
     Acx =Fx2 /m; %Longitudinal acceleration
end
    
VmaxAccel =sqrt(2*75*Acx )*3.6; %Maximum acceleration velocity in 75 m;
Long_force_coef =Fx2/(FZRR+FZRL);