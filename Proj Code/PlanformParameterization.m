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

classdef PlanformParameterization
    %{ The PlanformParameterization Object is designed to calculate Theoretical Wing/Horizontal Tail
    %  Theoretical Wing/HT Area, Aspect Ratio, Mean Chord Length, Y distance From Chord Root to Mean
    %  Aerodynamic Chord, X distance from Wing Apex to Centroid, X distance Chord From Wing Apex to 
    %  Mean Aerodynamic, Sweep at Quarter Chord,and Sweep at Half Chord
    %}
    properties
        Ct % Chord Tip Length (ft)
        Cr % Chord Root Length (ft)
        Lambda % Taper Ratio
        SSPN % Semi Span (ft)
        b % Span (ft)
        Sweep_LE % Leading Edge Sweep (degrees)
    end
    
    methods
        % Constructor to initialize properties
        function obj = PlanformParameterization(Ct, Cr, SSPN, Sweep_LE)
            obj.Ct = Ct;
            obj.Cr = Cr;
            obj.Lambda = Ct / Cr;
            obj.SSPN = SSPN;
            obj.b = 2 * SSPN;
            obj.Sweep_LE = Sweep_LE;
        end
        
        % Calculate Theoretical Wing Area
        function S = calcWingArea(obj)
            S = (obj.b / 2) * obj.Cr * (1 + obj.Lambda);
        end
        
        % Calculate Aspect Ratio
        function Aspect = calcAspectRatio(obj, S)
            Aspect = obj.b^2 / S;
        end
        
        % Calculate Mean Chord Length
        function C_bar = calcMeanChord(obj)
            C_bar = obj.Cr * (2 / 3) * ((1 + obj.Lambda + obj.Lambda^2) / (1 + obj.Lambda));
        end
        
        % Calculate Y distance From Chord Root to Mean Aerodynamic Chord
        function y_bar_mac = calcYBarMAC(obj)
            y_bar_mac = (obj.b / 6) * ((1 + (2 * obj.Lambda)) / (1 + obj.Lambda));
        end
        
        % Calculate sigma constant for calculations
        function sigma = calcSigma(obj)
            sigma = (obj.calcAspectRatio(obj.calcWingArea()) / 4) * (1 + obj.Lambda) * tand(obj.Sweep_LE);
        end
        
        % Calculate X distance from Wing Apex to Centroid
        function x_bar_centroid = calcXBarCentroid(obj)
            sigma = obj.calcSigma();
            x_bar_centroid = (1 / 3) * obj.Cr * (obj.Lambda + sigma + ((1 + (obj.Lambda * sigma)) / (1 + obj.Lambda)));
        end
        
        % Calculate X distance Chord From Wing Apex to Mean Aerodynamic 
        function x_bar_mac = calcXBarMAC(obj)
            y_bar_mac = obj.calcYBarMAC();
            x_bar_mac = y_bar_mac * tand(obj.Sweep_LE);
        end
         
        % Calculate Sweep at C/4
         function Sweep_C4 = calcSweepC4(obj)
            A = obj.calcAspectRatio(obj.calcWingArea());
            Sweep_C4 = atand(tand(obj.Sweep_LE) - (4 * 0.25 * (1 - obj.Lambda) / (A * (1 + obj.Lambda))));
        end
        
        % Calculate Sweep at Half Chord
        function Sweep_C2 = calcSweepC2(obj)
            A = obj.calcAspectRatio(obj.calcWingArea());
            Sweep_C2 = atand(tand(obj.Sweep_LE) - (4 * 0.5 * (1 - obj.Lambda) / (A * (1 + obj.Lambda))));
        end
    end
end
