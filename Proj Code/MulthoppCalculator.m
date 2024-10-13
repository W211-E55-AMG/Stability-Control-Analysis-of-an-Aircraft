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

classdef MulthoppCalculator
    % The MulthoppCalculator calculates the contribution of the fuselage 
    % and engine nacelles to the aircraft's moment coefficient change with
    % respect to the angle of attack, based on Multhopp's method. This method
    % is an adjustment to Munk's work, incorporating the wing's upwash and
    % downwash effects on the fuselage. By considering these aerodynamic
    % interactions, the MulthoppCalculator provides a more accurate assessment
    % of the fuselage's and engine nacelle's impact on the aircraft's stability
    % and control characteristics. Datcom's method is used to calculate
    % Cmof and Cmof'
    
     properties
        Sw % Wing area
        cw % Mean aerodynamic chord of the wing
        aw % Lift curve slope of the wing
        wfi %  Mean Width of the fuselage at various fwd stations of the wing's apex
        xi % Length of the segment cuts of the fuselage at various fwd stations of the wing's apex
        cre % Chord Root Exposed
        lf % Length of the fuselage
        alpha_0 % Alpha Zero Lift
        iw % Incedence angle of the wing
        iclb % Fuselage Camber Angle
        xix % % Position of the segment cuts of the fuselage at various fwd and aft stations of the wing's apex
        wfix % Mean Width along the fuselage at various fwd and aft segments
        Wfin % Widths of engine nacelle
        Xin % Positions of engine nacelle
        crn % Chord Root of engine nacelle in ft 
    end
    
    methods
        function obj = MulthoppCalculator(Sw, cw, aw, wfifwd, xifwd, cre, lf, alpha_0, iw, iclb,xi,wfi, Wfin, Xin, crn)
            obj.Sw = Sw;
            obj.cw = cw;
            obj.aw = aw;
            obj.wfi = wfi;
            obj.xi = xi;
            obj.wfix = wfifwd;
            obj.xix = xifwd;
            obj.cre = cre;
            obj.lf = lf;
            obj.alpha_0 = alpha_0;
            obj.iw = iw;
            obj.iclb = iclb;
            obj.Wfin = Wfin;
            obj.Xin = Xin;
            obj.crn = crn;
        end

        % Calculate dϵ/dα for the i-th section
        function d_epsilon_u_d_alpha = calculateDEpsilonUDAlpha(obj, i)
            if i < length(obj.xix)
                d_epsilon_u_d_alpha = (0.175 * (obj.xix(i) / obj.cre)^(-1.341)) * (obj.aw / 4.498);
            else % Use a different formula for the last section
                d_epsilon_u_d_alpha = (0.754 * (obj.xix(i) / obj.cre)^(-0.793)) * (obj.aw / 4.498);
            end
        end
        
        % Calculate C_m_alpha_f using Multhopp's method
        function C_m_alpha_f = calculateCmAlphaF(obj)
            n = length(obj.wfix); 
            sum_term = 0;
            for i = 1:n
                d_beta_d_alpha_i = 1 + obj.calculateDEpsilonUDAlpha(i);
                sum_term = sum_term + (obj.wfix(i)^2 * d_beta_d_alpha_i * obj.xix(i));
            end
            C_m_alpha_f = (pi / (2*obj.Sw*obj.cw)) * sum_term;
        end
        
        % Calculate Cmof and Cmof_prime using Datcom's method
        function Cmof_prime = calculateFuselageContributions(obj)
            e = sqrt(1 - (max(obj.wfi) / obj.lf)^2);
            alpha_0calc = 1/(e^2) - (1 - e^2)/(2*e^3) * log((1 + e)/(1 - e));
            K_2 = alpha_0calc/(2-alpha_0calc);
            beta_0 = ((2*(1 - e^2))/e^3) * (0.5 * log((1 + e)/(1 - e)) - e);
            K_1 = beta_0/(2-beta_0);
            
            sum_term = 0;
            for i = 1:length(obj.iclb)-1
                delta_x = obj.xi(i+1) - obj.xi(i);
                sum_term = sum_term + (obj.wfi(i)^2 * ((-obj.iw+obj.alpha_0) + obj.iclb(i)) * delta_x);
            end
            
            Cmof_prime = (K_2 - K_1) / (36.5 * obj.Sw * obj.cw) * sum_term;

        end

        function d_epsilon_u_d_alpha_n = calculateDEpsilonUDAlphaN(obj)
            
                d_epsilon_u_d_alpha_n = (0.754 * (obj.Xin / obj.crn)^(-0.793)) * (obj.aw / 4.498);
            
        end

        % Calculate Cmalpha_n for the engine nacelle
        function Cmalpha_n = calculateCmalphaN(obj)
            n = length(obj.Wfin);
            sum_term = 0;
            for i = 1:n
                d_beta_d_alpha_i = 1 + obj.calculateDEpsilonUDAlphaN();
                sum_term = sum_term + (obj.Wfin(i)^2 * d_beta_d_alpha_i * obj.Xin(i));
            end
            Cmalpha_n = pi / (2 * obj.Sw * obj.cw) * sum_term;
        end

    end
end
