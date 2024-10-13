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

classdef DownwashCalculator
    %{ 
    The DownwashCalculator is designed to calculate the actual downwash gradient 
    (dϵ/dα) on an aircraft. It uses aerodynamic and geometric parameters of the 
    wing and horizontal tail to determine the downwash effect experienced by the 
    tail due to the wing's presence. The calculation takes into account the incidence 
    angles of both the wing and the tail, ensuring an accurate representation of the 
    downwash effect based on the aircraft's geometry.
    %}

    properties
        Aw % Aspect ratio of the wing
        Lambda_c4 % Sweep angle at quarter chord (degrees)
        lambda_w % Taper ratio of the wing
        h_epsilon % Vertical distance from the wing TE to the horizontal tail quarter-chord
        bw % Wing span
        l_epsilon % Horizontal distance from the wing's TE to the horizontal tail's quarter-chord
    end
    
    methods
        function obj = DownwashCalculator(Aw, Lambda_c4, lambda_w, h_epsilon, bw, l_epsilon)
            obj.Aw = Aw;
            obj.Lambda_c4 = Lambda_c4;
            obj.lambda_w = lambda_w;
            obj.h_epsilon = h_epsilon;
            obj.bw = bw;
            obj.l_epsilon = l_epsilon;
        end

        % Calculate K_A
        function K_A = calculateKA(obj)
            K_A = (1 / obj.Aw) - (1 / (1 + obj.Aw^1.7));
        end

        % Calculate K_lambda 
        function K_lambda = calculateKLambda(obj)
            K_lambda = (10-(3*obj.lambda_w)) / 7;
        end
        
        % Calculate K_H
        function K_H = calculateKH(obj)
            K_H = 1 - (obj.h_epsilon / obj.bw) / (2 * obj.l_epsilon / obj.bw)^(1/3);
        end
        
        % Calculate the actual downwash gradient d_epsilon/d_alpha
        function d_epsilon_d_alpha = calculateDEpsilonDAlpha(obj)
            K_A = obj.calculateKA();
            K_lambda = obj.calculateKLambda();
            K_H = obj.calculateKH();
            d_epsilon_d_alpha = 4.44 * ((K_A * K_lambda * K_H * (cosd(obj.Lambda_c4))^0.5)^1.19);
        end
    end
end
