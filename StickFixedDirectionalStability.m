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

classdef StickFixedDirectionalStability
        % This object is designed to measure the directional stability
        % coefficients, which are essential for accounting for variables
        % that cause the plane to yaw, such as engine-out scenarios and
        % performing coordinated turns. It ensures a comprehensive analysis
        % of the aircraft's yaw stability under various conditions. 

    properties
        CL; % Lift coefficient
        Gamma; % Dihedral angle (degrees)
        Aw; % Wing aspect ratio
        Lambda_c4; % Wing sweep at quarter chord (degrees)
        xcg; % Center of gravity location as percent chord
        xac; % Aerodynamic center of the wing as percent chord
        KN; % Emperical factor related to sideslip derivative for Body + Wing-Body Interference from Datcom Fig 5.2.3.1-8 
        KRL; % Effect of Fuselage Reynolds Number on Wing Body from Datcom Fig 5.2.3.1-9
        Sw; % Wing area (ft^2)
        Lf; % Fuselage length (ft)
        bw; % Wing span (ft)
        CYbetaVeff % Effective vertical tail side force derivative from Datcom Fig 5.3.1.1-24b
        CYbetaRatio; % Ratio for Effective vertical tail side force derivative from Datcom Fig 5.3.1.1-24c
        Stvt; % Tail vertical area (ft^2)
        npe; % Number of prop engines
        CYbetaP; % Propeller contribution to side force derivative from Datcom Fig 4.6.1-25a
        Sp; % Propeller area (ft^2)
        xp; % X-coordinate of propeller
        f; % Propeller Inflow Factor from NACA WR L-25
        Zu; % Upper body coordinates array
        Zl; % Lower body coordinates array
        x;  % Chordwise stations array
        lv; % Distance from CG to vertical tail aerodynamic center (ft)
    end
    
    methods
        function obj = StickFixedDirectionalStability(CL, Gamma, Aw, Lambda_c4, xcg, xac, KN, KRL, Sw, Lf, bw, CYbetaVeff, CYbetaRatio, Stvt, npe, CYbetaP, Sp, xp, f, Zu, Zl, x,lv)
            obj.CL = CL;
            obj.Gamma = Gamma;
            obj.Aw = Aw;
            obj.Lambda_c4 = Lambda_c4*(pi/180);
            obj.xcg = xcg;
            obj.xac = xac;
            obj.KN = KN;
            obj.KRL = KRL;
            obj.Sw = Sw;
            obj.Lf = Lf;
            obj.bw = bw;
            obj.CYbetaRatio = CYbetaRatio;
            obj.CYbetaVeff = CYbetaVeff;
            obj.Stvt = Stvt;
            obj.npe = npe;
            obj.CYbetaP = CYbetaP;
            obj.Sp = Sp;
            obj.xp = xp;
            obj.f = f;
            obj.Zu = Zu;
            obj.Zl = Zl;
            obj.x = x;
            obj.lv = lv;
        end
        
        function CNbetaWGamma = calcCNbetaWGamma(obj)
            % Calculate C_NbetaWGamma
            CNbetaWGamma = -obj.CL * (0.075 / (57.3^2)) * obj.Gamma;
        end
        
        function CNbetaWLambdaM = calcCNbetaWLambdaM(obj)
            % Calculate C_NbetaW,Lambda@M
            term1 = (obj.CL^2 / 57.3);
            term2 = (1 / (4 * pi * obj.Aw));
            term3 = (tan(obj.Lambda_c4) / (pi * obj.Aw * (obj.Aw + 4 * cos(obj.Lambda_c4))));
            term4 = cos(obj.Lambda_c4) - (obj.Aw / 2) - ((obj.Aw^2) / (8 * cos(obj.Lambda_c4))) - ((6 / obj.Aw) * (obj.xcg - obj.xac) * sin(obj.Lambda_c4));
            CNbetaWLambdaM = term1 * (term2 - term3 * term4);
        end
        
        function SBS = calcSBS(obj)
            % Calculate S_BS
            n = length(obj.x);
            SBS = 0;
            for i = 1:(n-1)
                deltaZ = 0.5 * ((obj.Zu(i) - obj.Zl(i)) + (obj.Zu(i+1) - obj.Zl(i+1)));
                deltaX = obj.x(i+1) - obj.x(i);
                SBS = SBS + deltaZ * deltaX;
            end
        end

        function CNbetaBW = calcCNbetaBW(obj)
            Sbs = obj.calcSBS();
            % Calculate C_NbetaBW
            CNbetaBW = -obj.KN * obj.KRL * (Sbs / obj.Sw) * (obj.Lf / obj.bw);
        end
        
        function CYbetaVT = calcCYbetaVT(obj)
            % Calculate C_YbetaVT
            CYbetaVT = (obj.CYbetaVeff / 57.3) * (obj.CYbetaRatio) * ((2 * obj.Stvt) / obj.Sw);
        end
        
        function CNbetaVT = calcCNbetaVT(obj, CYbetaVT)
            % Calculate C_NbetaVT
            CNbetaVT = -CYbetaVT * (obj.lv / obj.bw);
        end
        
        function CNbetaProp = calcCNbetaProp(obj)
            % Calculate C_NbetaProp
            CNbetaProp = (obj.npe / 57.3)* obj.f * obj.CYbetaP * (obj.Sp / obj.Sw) * (obj.xp / obj.bw);
        end
        
        function CNbeta = calcCNbeta(obj)
            % Calculate total C_Nbeta
            CNbetaWGamma = obj.calcCNbetaWGamma();
            CNbetaWLambdaM = obj.calcCNbetaWLambdaM();
            CNbetaBW = obj.calcCNbetaBW();
            CYbetaVT = obj.calcCYbetaVT();
            CNbetaVT = obj.calcCNbetaVT(CYbetaVT);
            CNbetaProp = obj.calcCNbetaProp();
            CNbeta = CNbetaWGamma + CNbetaWLambdaM + CNbetaBW + CYbetaVT + CNbetaVT + CNbetaProp;
        end
    end
end
