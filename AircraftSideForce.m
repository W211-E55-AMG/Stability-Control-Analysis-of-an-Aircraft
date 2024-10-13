
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

classdef AircraftSideForce
        % The purpose of the AircraftSideForce Object is to calculate the
        % side force gradient on the aircraft. This gradient represents the
        % slope between the side force and the side slip angle, providing
        % data for understanding the aircraft's lateral aerodynamic characteristics.

    properties
        Sw  % Theoretical Wing Area (ft^2)
        So  % Body Station C-sec (ft) where the flow becomes Viscous Datcom figure 4.2.1.1-20b
        lf  % Fuselage length in (ft)
        wfi  % Fuselage widths at various stations
        CYbetaVeff  % Effective vertical tail side force derivative from Datcom Fig 5.3.1.1-24b
        CYbetaRatio  % Ratio for Effective vertical tail side force derivative from Datcom Fig 5.3.1.1-24c
        Stvt  % Theoretical Area of a single vertical fin (ft^2)
        npe  % Number of propeller engines
        f  % Propeller Inflow Factor from NACA WR L-25
        CYbetaP  % Propeller contribution to side force derivative from Datcom Fig 4.6.1-25a
        Sp  % Area affected by propellers
        gamma % Dihedral angle
        Kwb % Wing-Body Interference Factor for sideslip derivative from Datcom Fig 5.2.1.1-7 
    end
    
    methods
        function obj = AircraftSideForce(Sw, So, lf, wfi, CYbetaVeff, CYbetaRatio, Stvt, npe, f, CYbetaP, Sp, gamma, Kwb)
            obj.Sw = Sw;
            obj.So = So;
            obj.lf = lf;
            obj.wfi = wfi;
            obj.CYbetaVeff = CYbetaVeff;
            obj.CYbetaRatio = CYbetaRatio;
            obj.Stvt = Stvt;
            obj.npe = npe;
            obj.f = f;
            obj.CYbetaP = CYbetaP;
            obj.Sp = Sp;
            obj.gamma = gamma;
            obj.Kwb = Kwb;
        end
        
        % Calculate e, alpha_0calc, K_2, beta_0, and K_1
        function [K_1, K_2, e] = calculateKs(obj)
            % Calculate e, alpha_0calc, K_2, beta_0, and K_1
            e = sqrt(1 - (max(obj.wfi) / obj.lf)^2);
            alpha_0calc = 1/(e^2) - (1 - e^2)/(2*e^3) * log((1 + e)/(1 - e));
            K_2 = alpha_0calc/(2-alpha_0calc);
            beta_0 = ((2*(1 - e^2))/e^3) * (0.5 * log((1 + e)/(1 - e)) - e);
            K_1 = beta_0/(2-beta_0);
        end

        % Calculate C_Y_(β_WB)
        function CYbetaWB = calculateCYbetaWB(obj)
            [K_1, K_2]= obj.calculateKs();
            CYbetaWB = ((-2 / 57.3) * obj.Kwb * (K_2 - K_1) * (obj.So / obj.Sw)) - (0.0001 * abs(obj.gamma));
        end

        % Calculate C_YbetaVTw
        function CYbetaVT = calcCYbetaVT(obj)
            CYbetaVT = (obj.CYbetaVeff / 57.3) * obj.CYbetaRatio * ((2 * obj.Stvt) / obj.Sw);
        end

        % Calculate C_NbetaProp
        function CNbetaProp = calcCNbetaProp(obj)
            CNbetaProp = -(obj.npe / 57.3) * obj.f * obj.CYbetaP * (obj.Sp / obj.Sw);
        end

        % Calculate the total side force gradient
        function sideForceGradient = calculateSideForceGradient(obj)
            CYbetaWB = obj.calculateCYbetaWB();
            CYbetaVT = obj.calcCYbetaVT();
            CNbetaProp = obj.calcCNbetaProp();
            sideForceGradient = CYbetaWB + CYbetaVT + CNbetaProp;
        end
    end
end
