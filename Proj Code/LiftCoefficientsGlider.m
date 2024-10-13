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

classdef LiftCoefficientsGlider
    % This object is designed to compute the zero-lift coefficient (CLo)
    % and the lift curve slope (CLα​) for a glider, encapsulating the
    % aerodynamic contributions of both the wing and the horizontal tail.
    
    properties
        aw % Wing's lift curve slope
        iw % Wing's angle of incidence
        a0Lw % Zero-lift angle of attack for the wing
        at % Tail's lift curve slope
        neta_t % Dynamic Pressure Ratio at the tail
        St % Area of the horizontal tail
        Sw % Area of the wing
        it % Tail's angle of incidence
        d_epsilon_d_alpha % Downwash gradient
    end
    
    methods
        function obj = LiftCoefficientsGlider(aw, iw, a0Lw, at, neta_t, St, Sw, it, d_epsilon_d_alpha)
            obj.aw = aw;
            obj.iw = iw;
            obj.a0Lw = a0Lw;
            obj.at = at;
            obj.neta_t = neta_t;
            obj.St = St;
            obj.Sw = Sw;
            obj.it = it;
            obj.d_epsilon_d_alpha = d_epsilon_d_alpha;
        end
        
        function CL0 = calculateCL0(obj)
            CL0 = obj.aw * (obj.iw - obj.a0Lw) + obj.at * obj.neta_t * (obj.St / obj.Sw) * (obj.it - obj.d_epsilon_d_alpha * (obj.iw - obj.a0Lw));
        end
        
        function CLalpha = calculateCLalpha(obj)
            CLalpha = obj.aw + obj.at * obj.neta_t * (obj.St / obj.Sw) * (1 - obj.d_epsilon_d_alpha);
        end
    end
end
