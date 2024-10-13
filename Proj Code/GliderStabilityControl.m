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

classdef GliderStabilityControl
    % The GliderStabilityControl Object is designed to calculate the stability
    % of the aircraft (Cmαf​) based on Gilruth and White's (G&W) method. It also
    % determines the stick-fixed neutral point for both Cmαf calculated using G&W's 
    % method and Multhopp's method. This enables the user user to see the difference in
    % accuracy between both methods, which is crucial to take into account for stability
    % calculations to ensure the aircraft's control and stability characteristics are
    % accurately assessed.

    properties
        aw       % Lift curve slope of the wing
        xcg      % Center of gravity location as percent chord
        xacw     % Aerodynamic center of the wing as percent chord
        zw       % Vertical distance of wing from Fuselage Center Line
        cw       % Mean aerodynamic chord of the wing
        at       % Lift curve slope of the tail (per rad)
        eta_t    % Dynamic pressure ratio
        St       % Area of the tail ft^2
        Sw       % Wing area ft^2
        ew       % Span efficiency factor
        Aw       % Aspect ratio of the wing
        depsda   % Downwash gradient
        W        % Weight of the aircraft
        rho      % Air density (slug/cf)
        V        % True airspeed (Knots)
        XH       % Apex of Horizontal Tail ft
        XMACT    % Mean aerodynamic chord of the tail
        Xac_ht   % Aerodynamic Center Location of HT in (ft)
        Wfmax    % Maximum width of the fuselage (ft)
        Lf       % Length of the fuselage (ft)
        Cr       % Root chord of the wing (ft)
        CmAlpha_f_Multhopp % Cmalphaf from Multhopp's method
        XCG     % Center of Gravity location in (ft)
        XW      % Wing's Apex location in (ft)
        CLalpha  % Lift curve slope of the aircraft
        C_m_aplha_n % Cmalpha of the naccelle 
    end
    
    methods
        function obj = GliderStabilityControl(aw, xcg, xacw, zw, cw, at, eta_t, St, Sw, ew, Aw, depsda, W, rho, V, XH, XMACT, Xac_ht, Wfmax, Lf, Cr, CmAlpha_f_Multhopp, XCG, XW, CLalpha, C_m_aplha_n)
            obj.aw = aw;
            obj.xcg = xcg;
            obj.xacw = xacw;
            obj.zw = zw;
            obj.cw = cw;
            obj.at = at;
            obj.eta_t = eta_t;
            obj.St = St;
            obj.Sw = Sw;
            obj.ew = ew;
            obj.Aw = Aw;
            obj.depsda = depsda;
            obj.W = W;
            obj.rho = rho;
            obj.V = V;
            obj.XH = XH;
            obj.XMACT = XMACT;
            obj.Xac_ht = Xac_ht;
            obj.Wfmax = Wfmax;
            obj.Lf = Lf;
            obj.Cr = Cr;
            obj.CmAlpha_f_Multhopp = CmAlpha_f_Multhopp;
            obj.XCG = XCG;
            obj.XW = XW;
            obj.CLalpha = CLalpha;
            obj.C_m_aplha_n = C_m_aplha_n;
        end
        
        % Calculate distance from CG to tail aerodynamic center
        function lt = calclt(obj)
            lt = (obj.XH + obj.XMACT + obj.Xac_ht) - obj.XCG;
        end
        
        % Calculate fuselage influence factor
        function Kf = calcKf(obj)
            Kf = 0.0023322 * exp(5.037 * ((obj.XW + (obj.Cr * 0.25)) / obj.Lf));
        end
        
        % Calculate Coefficient of moment of fuselage using G&W method
        function Cm_alpha_f_GW = calcCm_alpha_f_GW(obj)
            Kf = obj.calcKf();
            Cm_alpha_f_GW = (Kf * obj.Wfmax^2 * obj.Lf) / (obj.Sw * obj.cw);
        end
        
        % Calculate CLw
        function CLw = calculateCLw(obj)
            Q = 0.5 * obj.rho * obj.V^2; % Dynamic pressure
            CLw = obj.W / (Q * obj.Sw);
        end
        
        % Calculate CmAlpha using CmAlpha,f from the G&W method
        function CmAlphaG = calculateCmAlpha_GW(obj)
            Cm_alpha_f_GW = obj.calcCm_alpha_f_GW(); 
            CLw = obj.calculateCLw(); 
            lt = obj.calclt(); 
            CmAlphaG = obj.C_m_aplha_n + Cm_alpha_f_GW + (obj.aw * (obj.xcg - obj.xacw)) ...
                + (obj.aw * CLw * ((1 / obj.aw) - (2 / (pi * obj.ew * obj.Aw))) * (obj.zw / obj.cw)) ...
                - (obj.at * obj.eta_t * ((obj.St * lt) / (obj.Sw * obj.cw)) * (1 - obj.depsda));
        end

        % Calculate CmAlpha using CmAlpha,f from Multhopp's method
        function CmAlphaM = calculateCmAlpha_Multhopp(obj)
            CLw = obj.calculateCLw(); 
            lt = obj.calclt(); 
            CmAlphaM = obj.C_m_aplha_n + obj.CmAlpha_f_Multhopp + (obj.aw * (obj.xcg - obj.xacw)) ...
                + (obj.aw * CLw * ((1 / obj.aw) - (2 / (pi * obj.ew * obj.Aw))) * (obj.zw / obj.cw)) ...
                - (obj.at * obj.eta_t * ((obj.St * lt) / (obj.Sw * obj.cw)) * (1 - obj.depsda));
        end

        % Calculate Neutral Point using G&W method
         function No_GW = calculateNeutralPoint_GW(obj)
            CmAlphaG = obj.calculateCmAlpha_GW()*(pi/180);
            No_GW = -(CmAlphaG / obj.CLalpha) + obj.xcg;
        end
        
        % Calculate Neutral Point using Multhopp's method
        function No_Multhopp = calculateNeutralPoint_Multhopp(obj)
            CmAlphaM = obj.calculateCmAlpha_Multhopp()*(pi/180); 
            No_Multhopp = -(CmAlphaM / obj.CLalpha) + obj.xcg;
        end
    end
end
