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

classdef PowerEffects
        % This object is designed to assess the aircraft's stability by
        % calculating its new stick-fixed neutral point when power is
        % applied, considering both windmilling and full power scenarios.
        % Additionally, the full power case takes into account the specific
        % aircraft type, providing a comprehensive analysis of stability
        % under different power conditions.

    properties
        npe % Number of Engines with Props
        CNap % Propeller Normal Force Curve Slope at Kn=90 and Tc = 0
        Sp % Propeller affected area (ft^2)
        Sw % Wing area (ft^2)
        dEpsdAlpha % Change in downwash with angle of attack
        aw % Wing lift curve slope (per rad)
        Swi % Wing area immersed in the prop slipstream
        at % Tail lift curve slope (per rad)
        nT % Tail efficiency
        SH % Tail area (ft^2)
        xp % Distance from wing reference point to propeller
        xwi % Longitudinal distance from CG fwd to ac in refrence to the chord immersed in the prop slipstream
        cw % Mean aerodynamic chord of the wing
        lt % Distance from CG to tail aerodynamic center (ft)
        xcg % Center of gravity location as percent chord
        del_fp % Correction factor for stick fixed nuetral point inregards to the full power case & aircraft type
               % From Perkins and Hage per NACA WR L-24
    end
    
    methods
        function obj = PowerEffects(npe, CNap, Sp, Sw, dEpsdAlpha, aw, Swi, at, nT, SH, xp, xwi, cw, lt,xcg)
            obj.npe = npe;
            obj.CNap = CNap;
            obj.Sp = Sp;
            obj.Sw = Sw;
            obj.dEpsdAlpha = dEpsdAlpha;
            obj.aw = aw;
            obj.Swi = Swi;
            obj.at = at;
            obj.nT = nT;
            obj.SH = SH;
            obj.xp = xp;
            obj.xwi = xwi;
            obj.cw = cw;
            obj.lt = lt;
            obj.xcg = xcg;
            obj.del_fp = -.08;
        end
        
        % Calculate Change in CLα inregards Windmilling power condition
        function dCLalphaWP = deltaCLalphaWP(obj)
            dCLalphaWP = ((obj.npe / 57.3) * obj.CNap * (obj.Sp / obj.Sw) * (1 + obj.dEpsdAlpha)) ...
                         - (obj.npe * obj.aw * (obj.Swi / obj.Sw) * (0.25 * obj.CNap)) ...
                         + (obj.at * obj.nT * (obj.SH / obj.Sw) * (0.25 * obj.CNap));
        end
        
        % Calculate Full power Change in CMα inregards Windmilling power condition
        function dCMalphaWP = deltaCMalphaWP(obj)
            dCMalphaWP = ((obj.npe / 57.3) * obj.CNap * ((obj.Sp * obj.xp) / (obj.Sw * obj.cw)) * (1 + obj.dEpsdAlpha)) ...
                         - (obj.npe * obj.aw * ((obj.Swi * obj.xwi) / (obj.Sw * obj.cw)) * (0.25 * obj.CNap))...
                         + (obj.at * obj.nT * ((obj.SH * obj.lt) / (obj.Sw * obj.cw)) * (0.25 * obj.CNap));
        end
        
        
        % Calculate Stick-Fixed Nuetral Point for both Conditions
        function [No_wp, No_fp]= NeutralPoint(obj)
            dCLalphaWP = obj.deltaCLalphaWP();
            CMalphaWP = obj.deltaCMalphaWP();
            No_wp = -(CMalphaWP / dCLalphaWP)*(pi/180) + obj.xcg;
            No_fp = No_wp + obj.del_fp;
        end
    end
end

