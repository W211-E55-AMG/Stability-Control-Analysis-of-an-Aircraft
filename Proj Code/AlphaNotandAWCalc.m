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

classdef AlphaNotandAWCalc
    %{ The AlphaNotCalc Object calculates the lift curve slope and the 
    % angle of attack at which zero lift is produced (Alpha Not L), 
    % both using Datcom's Equation. 
    %}
    properties
        A % Aspect Ratio
        a_not % Experimental Lift Curve Slope (degrees)
        Alpha_notL_0Theta % Alpha Not L when wing has no Wash(out/in)
        Mach % Mach Number
        TWISTA % Washin/WashOut
        Del_alpha_Theta % Change in Alpha due to Twist from Datcom Fig 4.1.3.1-4
        Comp_Corr % Compressibility Correction factor for alpha not Datcom Fig 4.1.3.1-5
        Sweep_C2 % Sweep at Half Chord
    end
    
    methods

        function obj = AlphaNotandAWCalc(A, a_not, Alpha_notL_0Theta, Mach, TWISTA, Del_alpha_Theta, Comp_Corr, Sweep_C2)
            obj.A = A;
            obj.a_not = a_not*(180/pi);
            obj.Alpha_notL_0Theta = Alpha_notL_0Theta;
            obj.Mach = Mach;
            obj.TWISTA = TWISTA;
            obj.Del_alpha_Theta = Del_alpha_Theta;
            obj.Comp_Corr = Comp_Corr;
            obj.Sweep_C2 = Sweep_C2;
        end
        
        % Calculate Beta (Prandtl Glauert's Compressibility Correction Factor)
        function beta = calcBeta(obj)
            if obj.Mach > 0.3
            beta = sqrt(1 - obj.Mach^2);
            else
            beta = 1;
            end
        end
        
        % Calculate K
        function k = calcK(obj)
            k = obj.a_not / (2 * pi);
        end
        
        %Calculate Lift Curve Slope Datcom Equation
        function a_correction = calcACorrection(obj)
            beta = obj.calcBeta();
            k = obj.calcK();
            a_correction = (2 * pi * obj.A) / (2 + sqrt( (((beta * obj.A) / k)^2 * (1 + ((tand(obj.Sweep_C2))^2 / beta^2))) + 4));
        end
        
        % Calculate Alpha Not L Datcom Equation
        function Alpha_NotL = calcAlphaNotL(obj)
            Alpha_NotL = (obj.Alpha_notL_0Theta + (obj.Del_alpha_Theta * obj.TWISTA)) * obj.Comp_Corr;
        end
    end
end
