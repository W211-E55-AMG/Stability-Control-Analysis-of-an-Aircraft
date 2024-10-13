% Project Group Code

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{
The source code contained herein was developed for a stability and control project at 
Embry-Riddle Aeronautical University for Professor Glenn P. Greiner, by Jose A. Rocha-Puscar, 
an undergraduate student of Aerospace Engineering at the Daytona Beach Campus. Copyright 2024. 
All rights reserved.

Although due care has been taken to present accurate programs, this software is provided "as is" 
WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED 
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE. The entire risk as to the quality 
and performance of the software is with the user. The program is made available only for educational 
and personal research purposes. It may not be sold to other parties. If you copy some or all of the 
software, you are requested to return a copy of any source additions that you believe make a significant
improvement in its range of application.
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%{ 
The main script serves as the central hub for the stability and control project. It gathers all necessary 
inputs required for the object calculations. The script contains the core object as a function,that performs the 
required calculations to achieve the desired outcomes. Finally, the script outputs the results of these 
computations into the command window, providing a clear and concise presentation of the findings.
%}



clc
clear
close all

%% Wing Paramerterization

% PlanformParameterization Inputs: 
Ct_w = 3.5; % Chord Tip in (ft)
Cr_w = 11.2597; %Chord Root in (ft)
SSPN_w = 23.7917; %Semi-Span in (ft)
Sweep_LE_w = 11.5479; %Sweep at the Leading Edge in (degrees)

Wing1 = PlanformParameterization(Ct_w, Cr_w, SSPN_w, Sweep_LE_w);

fprintf('Wing Parameterization Section\n\n')
% Theoretical Wing Area
Sw = Wing1.calcWingArea();
fprintf('Theoretical Wing Area: %.4f ft^2\n', Sw);

% Calculate Aspect Ratio
Aw = Wing1.calcAspectRatio(Sw);
fprintf('Aspect Ratio: %.4f\n', Aw);

% Mean Chord Length
C_barw = Wing1.calcMeanChord();
fprintf('Mean Chord : %.4f ft\n', C_barw);


% Y distance from Chord Root to MAC
y_bar_macw = Wing1.calcYBarMAC();
fprintf('Y distance from Chord Root to MAC: %.4f ft\n', y_bar_macw);

% X distance from Wing Apex to Centroid
x_bar_centroidw = Wing1.calcXBarCentroid();
fprintf('X distance from Wing Apex to Centroid: %.4f ft\n', x_bar_centroidw);

% X distance from Wing Apex to MAC
x_bar_macw = Wing1.calcXBarMAC();
fprintf('X distance from Wing Apex to MAC: %.4f ft\n', x_bar_macw);

% Sweep at Quarter Chord
Sweep_C4w = Wing1.calcSweepC4();
fprintf('Sweep at Quarter Chord: %.4f degrees\n', Sweep_C4w);

% Sweep at Half Chord
Sweep_C2w = Wing1.calcSweepC2();
fprintf('Sweep at Half Chord: %.4f degrees\n', Sweep_C2w);


% AlphaNotandAWCalc Inputs: 
Aw; % Aspect Ratio  
a_not = .104; % Experimental Lift Curve Slope (degrees)  
Alpha_notL_0Theta = -1.2; % Alpha Not L when wing has no Wash(out/in) from Table G.1 Datcom
Mach = .2873; % Mach Number
TWISTA = -3.53; % Wing twist angle (degrees), also known as wash-in or wash-out
Del_alpha_Theta = -.3975; % Change in Alpha due to Theta from Datcom Fig 4.1.3.1-4
Comp_Corr = 1; % Compressibility Correction factor from Datcom Fig 4.1.3.1-5
Sweep_C2w; % Sweep at Half Chord in degrees

Wing2 = AlphaNotandAWCalc(Aw, a_not, Alpha_notL_0Theta, Mach, TWISTA, Del_alpha_Theta, Comp_Corr, Sweep_C2w);

% Datcom's eq to account for subsonic Lift curve slope wrt a_not, Aspect
% Ratio, Mach #, and sweep
a_datcomw = Wing2.calcACorrection();
fprintf('Lift Curve Slope Datcom Eq: %.4f per deg\n', a_datcomw*(pi/180));

% Alpha Not L
Alpha_NotL = Wing2.calcAlphaNotL();
fprintf('Alpha Not L: %.4f\n', Alpha_NotL);

% CmacCalculation Inputs:
X_cr_ac = .38; % X coordinate wing aerodynamic center position from Datcom figure 4.1.4.2-26
x_bar_macw; % X distance from Wing Apex to MAC
Cmac = -.005; % Coefficient of moment at the aerodynamic center from Table G.1 Datcom
Sweep_C4w; % Sweep at the quarter chord (degrees)
Aw; % Aspect Ratio
Del_Cmnot_theta = -.0015; % Change in moment coefficient due to twist from Datcom figure 4.1.4.1-5
Comp_Corr; % Compressibility correction factor from datcom figure 4.1.4.1-6
C_barw; % % Mean Chord Length (ft)
TWISTA; % Wash(out/in) angle of twist (degrees)
Cr_w; % Chord Root (ft)

Wing3 = CmacCalculation(X_cr_ac, x_bar_macw, Cmac, Sweep_C4w, Aw, Del_Cmnot_theta, Comp_Corr, C_barw, TWISTA, Cr_w);

%Corrected Cmac
Cmac_Corrected = Wing3.calcCmacCorrected();
fprintf('Corrected Cmac for Twist Angle and Mach affects: %.4f\n', Cmac_Corrected);

%X position of Mean Aerodynamic Center in respect to Chord
XMACW = Wing3.calcX_mac();
fprintf('The Location of the Aerodynamic Center is %.4f%% of C_Bar\n',XMACW*100);


%% Hortizontal Tail Paramerterization

% PlanformParameterization Inputs: 
Ct_h = 4.5; % Hortizontal Tail Chord Tip in ft
Cr_h = 5.8333; % Hortizontal Tail Chord Root in ft
SSPN_h = 7.5039; % Hortizontal Tail Semi-Span in ft
Sweep_LE_h = 11.5; % Hortizontal Tail Sweep at the Leading Edge in degrees

Horizontal_Tail = PlanformParameterization(Ct_h, Cr_h, SSPN_h, Sweep_LE_h);

fprintf('\nHorizontal Tail Parameterization Section\n\n')
% Theoretical Wing Area
S_ht = Horizontal_Tail.calcWingArea();
fprintf('Horizontal Tail Theoretical Area: %.4f ft^2\n', S_ht);

% Aspect Ratio
A_ht = Horizontal_Tail.calcAspectRatio(S_ht);
fprintf('Aspect Ratio: %.4f\n', A_ht);

% Mean Chord Length (ft)
C_bar_ht = Horizontal_Tail.calcMeanChord();
fprintf('Mean Chord: %.4f ft\n', C_bar_ht);

% Y distance from Chord Root to Mean Aerodynamic
y_bar_mac_ht = Horizontal_Tail.calcYBarMAC();
fprintf('Y distance to MAC from Chord Root: %.4f ft\n', y_bar_mac_ht);

% X distance from Wing Apex to C_bar
x_bar_centroid_ht = Horizontal_Tail.calcXBarCentroid();
fprintf('X distance to Centroid from Wing Apex: %.4f ft\n', x_bar_centroid_ht);

% X distance from Wing Apex to Mean Aerodynamic Chord
x_bar_mac_ht = Horizontal_Tail.calcXBarMAC();
fprintf('X distance to MAC from Wing Apex: %.4f ft\n', x_bar_mac_ht);

% Sweep at Quarter Chord for Horizontal Tail
Sweep_C4_ht = Horizontal_Tail.calcSweepC4();
fprintf('Sweep at Quarter Chord: %.4f degrees\n', Sweep_C4_ht);

% Sweep at Half Chord for Horizontal Tail
Sweep_C2_ht = Horizontal_Tail.calcSweepC2();
fprintf('Sweep at Half Chord: %.4f degrees\n', Sweep_C2_ht);

% AlphaNotandAWCalc Inputs: 
A_ht; % Aspect Ratio  
a_not = .095; % Experimental Lift Curve Slope (degrees) from Table G.1 Datmcom 
Alpha_notL_0Theta = 0; % Alpha Not L when wing has no Wash(out/in) from Table G.1 Datcom 
Mach; % Mach Number
TWISTA_h = 0; % Washin/Washout
Del_alpha_Theta = -.431; % Change in Alpha due to Theta from Datcom Fig 4.1.3.1-4
Comp_Corr; % Compressibility Correction factor from Datcom Fig 4.1.3.1-5
Sweep_C2_ht; % Sweep at Half Chord (degrees)


Horizontal_Tail2 = AlphaNotandAWCalc(A_ht, a_not, Alpha_notL_0Theta, Mach, TWISTA_h, Del_alpha_Theta, Comp_Corr, Sweep_C2_ht);

% Datcom's eq to account for subsonic lift curve slope to take into account a_not, Aspect
% Ratio, Mach #, and sweep
a_datcomht = Horizontal_Tail2.calcACorrection();
fprintf('Lift Curve Slope Datcom Eq: %.4f per deg\n', a_datcomht*(pi/180));

% Theoretical DownWash Gradient
TheoroDownwashGradient = (2*a_datcomw)/(.985*pi*Aw); %.985 is the span efficiency factor from McCormick's Fig 4.22
fprintf('Theroetical Downwash Gradient is %.4f\n',TheoroDownwashGradient)

% Geometric Approach for Downwash Gradient
Aw; % Aspect ratio of the wing
Sweep_C4w; % Sweep angle at quarter chord (degrees)
lambda_w = .3108; % Taper ratio of the wing
h_epsilon = 2.6; % Vertical distance from the wing to the horizontal tail
bw =2*SSPN_w; % Wing span
l_epsilon = 20.6891; % Horizontal distance from the wing's quarter-chord to the horizontal tail's quarter-chord

% Actual Downwash Gradient Based of Datcom Sec 4.4.1
ADWGC = DownwashCalculator(Aw, Sweep_C4w, lambda_w , h_epsilon, bw, l_epsilon);

actualDownwashGradient = ADWGC.calculateDEpsilonDAlpha();
fprintf('Actual Downwash Gradient is %.4f\n',actualDownwashGradient)

%X position of Mean Aerodynamic Center in respect to Chord
fprintf('The Location of the Aerodynamic Center is %2.f%% of C_Bar\n',.25*100);


%% No Power Stability Calculations

fprintf('\nStability of Aircraft and its Components Section as a Glider\n\n');

% MulthoppCalculator Inputs:
Sw; % Wing area
C_barw; % Mean aerodynamic chord of the wing
a_datcomw; % Lift curve slope of the wing
wfifwd = [1.3, 2.9, 3.45, 3.85, 4.075, 4.275]; %  Mean Width of the fuselage at various fwd stations of the wing's apex
xifwd = [1.2, .8, .8, .8, .8, .8]; % Length of the segment cuts of the fuselage at various fwd stations of the wing's apex
cre = 11.1; % Chord Root Exposed
Lf = 32; % Length of the fuselage
Alpha_NotL; % Alpha Zero Lift
iw = 3.92; % Incidence angle of the wing
iclb = [-8, -7.5, -10, -6.5, 0, 0, 2.5, 4, 6, 6, 7, 7, 5, 5, 6]; % Fuselage Camber Angle at each segment
xi = [.8333, 1.5, 1.5, 2.083, 2.625, 2.6667, 2.6667, 2.625, 2.4583, 2.3333, 2.3333, 2.3333, 2.3333, 1.7917, 1.4583]; % Position of the segment cuts of the fuselage at various fwd and aft stations of the wing's apex
wfi = [1.225, 2.85, 3.75 , 4.25, 4.6, 4.7, 4.7, 4.7, 4.6, 4.2, 3.55, 2.9, 2.3, 1.8, 1.425]; % Mean Width along the fuselage at various fwd and aft stations
Wfin = 3.93; % Widths of engine nacelle
Xin = 4.6; % Positions of engine nacelle
crn = 8.8839; % Chord Root of engine nacelle in ft

% Create an instance of MulthoppCalculator
multhoppCalc = MulthoppCalculator(Sw, C_barw, a_datcomw, wfifwd, xifwd, cre, Lf, Alpha_NotL, iw, iclb, xi, wfi, Wfin, Xin, crn);

% Calculate C_m_alpha_f & C_malpha_n
C_m_alpha_n = 2 * multhoppCalc.calculateCmalphaN();
C_m_alpha_f = multhoppCalc.calculateCmAlphaF();

% Calculate fuselage contributions to obtain Cmof and Cmof_prime
Cmof_prime = multhoppCalc.calculateFuselageContributions();
Cmof = Cmof_prime(end);
% Display results

fprintf('Multhopp''s Method:\n')
fprintf('Cm_alpha_f for the Model 18 glider: %.4f per deg\n', C_m_alpha_f*(pi/180));
fprintf('Cm_alpha_n for the Model 18 glider: %.4f per deg\n', C_m_alpha_n*(pi/180));


%{ LiftCoefficientsGlider Inputs:
a_datcomw; % Wing's lift curve slope (per radians)
iw; % Wing's angle of incidence (degrees)
Alpha_NotL; % Zero-lift angle of attack for the wing (degrees)
a_datcomht; % Tail's lift curve slope (per radians)
neta_t = .9; % Dynamic Pressure Ratio at the tail
S_ht; % Area of the horizontal tail (ft^2)
Sw; % Area of the wing (ft^2)
it = -2; % Tail's angle of incidence (degrees)
actualDownwashGradient; % Downwash gradient
%}


% Create an instance of LiftCoefficientsGlider with the specified inputs
gliderCoeffCalc = LiftCoefficientsGlider(a_datcomw, iw, Alpha_NotL, a_datcomht, neta_t, S_ht, Sw, it, actualDownwashGradient);

% Calculate C_L_0 and C_L_alpha using the object's methods
CLo = gliderCoeffCalc.calculateCL0();
C_L_alpha = gliderCoeffCalc.calculateCLalpha()*(pi/180);


% GliderStabilityControl Inputs:
a_datcomw; % Lift curve slope of the wing
xcg = .2722; % Center of gravity location as percent chord
XMACW; % Aerodynamic center of the wing as percent chord
zw = -0.95; % Vertical distance of wing from Fuselage Center Line
C_barw; % Mean aerodynamic chord of the wing
a_datcomht; % Lift curve slope of the tail (per rad)
eta_t = 0.90; % Dynamic pressure ratio
S_ht; % Area of the tail ft^2
Sw; % Wing area ft^2
ew = .985; % Span efficiency factor
Aw; % Aspect ratio of the wing
actualDownwashGradient; % Downwash gradient
W = 8750; % Weight of the aircraft
rho = 17.56*10^-4; % Air density slug/cf
V = 309.467; % True airspeed Knots
XH = 28.18; % Apex of Horizontal Tail ft
x_bar_mac_ht; % Mean aerodynamic chord of the tail
Xac_ht = .25*C_bar_ht; % Aerodynamic Center Location of HT (ft)
Wfmax = 4.7; % Maximum width of the fuselage (ft)
Lf = 32; % Length of the fuselage (ft)
Cr_w; % Root chord of the wing (ft)
C_m_alpha_f; % Cmalphaf from Multhopp's method
XCG = 9.44; % Center of Gravity location in (ft)
XW = 5.2416; % Wing's Apex location in (ft)
C_L_alpha; % Lift curve slope of the aircraft
C_m_alpha_n; % Cmalpha of the naccelle

gliderControl = GliderStabilityControl(a_datcomw, xcg, XMACW, zw, C_barw, a_datcomht, eta_t, S_ht, Sw, ew, Aw, actualDownwashGradient, ...
    W, rho, V, XH, x_bar_mac_ht, Xac_ht, Wfmax, Lf, Cr_w, C_m_alpha_f, XCG, XW,C_L_alpha, C_m_alpha_n);

CLw = gliderControl.calculateCLw();
% Calculate CmAlpha using G&W method
CmAlpha_GW = gliderControl.calculateCmAlpha_GW();
No_GW = gliderControl.calculateNeutralPoint_GW();

% Using the previously calculated C_m_alpha_f from MulthoppCalculator as an example
% Assuming C_m_alpha_f has been calculated from MulthoppCalculator and provided here
CmAlpha_Multhopp = gliderControl.calculateCmAlpha_Multhopp();
No_Multhopp = gliderControl.calculateNeutralPoint_Multhopp();

% Display results

fprintf('Cm_alpha using Multhopp method for Cmalpha_f: %.4f\n', CmAlpha_Multhopp*(pi/180));
fprintf('The Nuetral Point using Multhopp method for Cmalpha_f: %.4f\n', No_Multhopp);


fprintf('\nGilruth and White''s Method:\n')
fprintf('Cm_alpha using G&W method for Cmalpha_f: %.4f\n', CmAlpha_GW*(pi/180));
fprintf('The Nuetral Point using G&W method for Cmalpha_f: %.4f\n', No_GW);

fprintf('\nNomarl Linearized C_L_alpha Eq\n')
fprintf('C_L_alpha for a glider Model 18 configuration is: %.4f per deg\n', C_L_alpha);

%% Power Stability Calculations
fprintf('\nStability of Aircraft and its Components Section for windmilling & full power\n\n');


%PowerEffects Inputs:
npe =  2; % Number of Engines with Props
CNap =  .120; % Propeller Normal Force Curve Slope at Kn=90 and Tc = 0
Sp =  53.4562; % Propeller affected area (ft^2)
Sw; % Wing area (ft^2)
dEpsdAlpha =  1.12; % Change in downwash with angle of attack
a_datcomw; % Wing lift curve slope (per rad)
Swi =  73.3425; % Wing area immersed in the prop slipstream (ft^2)
a_datcomht; % Tail lift curve slope (per rad)
eta_t; % Dynamic pressure ratio
S_ht; % Tail area (ft^2)
xp =  6.82; % Distance from wing reference point to propeller
xwi =  .3876; % Longitudinal distance from CG fwd to ac in refrence to the chord immersed in the prop slipstream
C_barw; % Mean aerodynamic chord of the wing ft
lt = gliderControl.calclt();% Distance from CG to tail aerodynamic center ft
xcg = .2722; % Center of gravity location as percent chord

windEffects = PowerEffects(npe, CNap, Sp, Sw, dEpsdAlpha, a_datcomw, Swi, a_datcomht, eta_t, S_ht, xp, xwi, C_barw, lt, xcg);
dCMalphaWP = windEffects.deltaCMalphaWP();
dCLalphaWP = windEffects.deltaCLalphaWP();

% Calculate the neutral points for wind milling power condition, both minimal and full power
[No_wp, No_fp] = windEffects.NeutralPoint();
fprintf('The neutral point under minimal wind milling power (No_wpm) is: %.4f\n', No_wp);
fprintf('The neutral point under full wind milling power (No_wpf) is: %.4f\n', No_fp);

%% Side Force Gradient
fprintf('\nSide Force Gradient Calculation \n');

% AircraftSideForce Inputs:
Sw; % Theoretical Wing Area (ft^2)
So = 17.3494; % Body Station C-sec (ft) where the flow becomes Viscous Datcom figure 4.2.1.1-20b
Lf; % Fuselage length in (ft)
wfi; % Fuselage widths at various stations
CYbetaVeff = 3.2; % Effective vertical tail side force derivative from Datcom Fig 5.3.1.1-24b
CYbetaRatio = .78; % Ratio for Effective vertical tail side force derivative from Datcom Fig 5.3.1.1-24c
Stvt = 8.5584; % Theoretical Area of a single vertical fin (ft^2)
npe; % Number of propeller engines
f = 1; % Propeller Inflow Factor from NACA WR L-25
CYbetaP = .12; % Propeller contribution to side force derivative from Datcom Fig 4.6.1-25a
Sp = 53.4562; % Wing Area affected by propellers in (ft^2)
Gamma = 6; % Dihedral angle (degrees)
Kwb = 1.5; % Wing-Body Interference Factor for sideslip derivative from Datcom Fig 5.2.1.1-7 

% Instantiate the AircraftSideForce class with the above values
aircraft = AircraftSideForce(Sw, So, Lf, wfi, CYbetaVeff, CYbetaRatio, Stvt, npe, f, CYbetaP, Sp, Gamma, Kwb);

% Calculate the side force gradient
sideForceGradient = aircraft.calculateSideForceGradient();

% Display the result
fprintf('The calculated side force gradient is: %.4f per deg\n', sideForceGradient);



%% Stick-Fixed Directional Stability
fprintf('\nStick-Fixed Directional Stability\n');

%StickFixedDirectionalStability Inputs
CLw; % Lift coefficient 
Gamma; % Dihedral angle (degrees)
Aw; % Wing aspect ratio
Sweep_C4w; % Wing sweep at quarter chord (degrees)
xcg; % Center of gravity location as percent chord
XMACW; % Aerodynamic center of the wing as percent chord
KN = 0.001; % Emperical factor related to sideslip derivative for Body + Wing-Body Interference from Datcom Fig 5.2.3.1-8 
KRL = 1.79; % Effect of Fuselage Reynolds Number on Wing Body from Datcom Fig 5.2.3.1-9
Sw; % Wing area (ft^2)
Lf; % Fuselage length (ft) 
bw; % Wing span (ft)
CYbetaVeff; % Effective vertical tail side force derivative from Datcom Fig 5.3.1.1-24b
CYbetaRatio; % Ratio for Effective vertical tail side force derivative from Datcom Fig 5.3.1.1-24c
Stvt; % Theoretical Area of a single vertical fin in ft^2
npe; % Number of prop engines
CYbetaP; % Propeller contribution to side force derivative from Datcom Fig 4.6.1-25a
Sp; % % Wing Area affected by propellers in ft^2
xp = 6.82; % Distance Prop Disc Plane to Center of Gravity in ft
f; % Propeller Inflow Factor from NACA WR L-25
Zu = [0.00,1.10,1.70,2.05,3.30,3.50,3.60,3.60,3.60,3.60,3.44,3.31,3.15,3.02,2.90,2.52,2.70]; % Upper body coordinates array
Zl = [0.00,-0.95,-1.50,-1.80,-2.05,-2.15,-2.10,-1.90,-1.70,-1.23,-0.74,-0.33, 0.17, 0.57, 0.95, 1.20, 2.70]; % Lower body coordinates array
xi = [0.00, .8333, 2.333, 3.8333, 6.0416, 8.6666,11.3333,14.00,16.625,19.0833,21.4166,23.7499,26.0832,28.4165,30.2082,31.665,33.8];  % Chordwise stations array
lv = 6.79; % Distance from CG to vertical tail aerodynamic center (ft)


aircraftDirectional = StickFixedDirectionalStability(CLw, Gamma, Aw, Sweep_C4w, xcg, XMACW, KN, KRL, Sw, Lf, bw, CYbetaVeff, CYbetaRatio, Stvt, npe, CYbetaP, Sp, 20.81, 1, Zu, Zl, xi, lv);

% Calculate C_Nbeta
CNbeta = aircraftDirectional.calcCNbeta();

 
% Display the result
fprintf('C_Nbeta for the aircraft configuration is: %.4f per deg\n', CNbeta);


%% Stick- Fixed Lateral Stability
fprintf('\nStick-Fixed Lateral Stability\n');

%StickFixedLateralStability Inputs:
CLw; % Lift coefficient
Cl_beta_over_CL_Aw =  -0.0002; % Aspect Ratio from Datcom Fig 5.1.2.1-28b
KM_Lambda =  1; % Compressibility Correction Factor to sweep contribution to wing's lateral stability from Datcom Fig 5.1.2.1-28a
KM_Gamma =  1; % Compressibility Correction Factor to  dihedral on wing's lateral stability from Datcom Fig 5.1.2.1-30a
Kf =  .99; % Fuselage Correction Factor rom Datcom Fig 5.2.2.1-26
Gamma; % Dihedral angle (degrees)
Sweep_C4w; % Wing sweep at quarter chord (degrees)
TWISTA; % Twist angle (degrees)
DeltaCl_beta_theta =  -0.000032; % Change in Cl_beta due to twist
Aw; % Wing aspect ratio
deq =  8.95; % Equivalent fuselage depth (ft)
zwprime =  1.17; % Distance from fuselage center line to wing root c/4 (ft)
bw; % % Wing span (ft)
CYbetaVT = aircraftDirectional.calcCYbetaVT(); % Side force coefficient due to vertical tail
hv =  4.76; % Difference in height from CG to vertical tail aerodynamic center (ft)
Cl_beta_over_CL_C2 =  -.0003; % Wing Sweep contribution to Lateral Stability From Datcom fig 5.1.2.1-27
Clb_Gamma_factor =  -0.0002; % Effect of Uniform Geometric Dihedral on Wing's Lateral Stability from Datcom Fig 5.1.2.1-29

aircraftLateral = StickFixedLateralStability(CLw, Cl_beta_over_CL_Aw, KM_Lambda, KM_Gamma, Kf, Gamma, Sweep_C4w, 3.53, ...
    DeltaCl_beta_theta, Aw, deq, zwprime, bw, CYbetaVT, hv, Cl_beta_over_CL_C2, Clb_Gamma_factor);

Clbeta = aircraftLateral.calcCl_beta();

fprintf('C_lbeta for the aircraft configuration is: %.4f per deg\n', Clbeta);



