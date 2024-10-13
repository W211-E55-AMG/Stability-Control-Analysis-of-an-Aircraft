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

classdef StickFixedLateralStability
        % The purpose of this object is to calculate the lateral stability 
        % coefficient, which represents the necessary adjustments the aircraft
        % needs to make to maintain level wings.

    properties
        CL; % Lift coefficient
        Cl_beta_over_CL_Aw; % Aspect Ratio from Datcom Fig 5.1.2.1-28b
        Cl_beta_over_CL_C2; % % Wing Sweep contribution to Lateral Stability From Datcom fig 5.1.2.1-27
        KM_Lambda; % Compressibility Correction Factor to sweep contribution to wing's lateral stability from Datcom Fig 5.1.2.1-28a
        KM_Gamma; % Compressibility Correction Factor to  dihedral on wing's lateral stability from Datcom Fig 5.1.2.1-30a
        Kf; % Fuselage Correction Factor rom Datcom Fig 5.2.2.1-26
        Gamma; % Dihedral angle
        Lambda_c; % Wing sweep at quarter chord
        theta; % Twist angle
        DeltaCl_beta_theta; % Effect of wing twist on wing's lateral stability Datcom Fig 5.1.2.1-30b
        Aw; % Wing aspect ratio
        deq; % Equivalent fuselage depth (ft)
        zw; % Distance from fuselage center line to wing root c/4 (ft)
        bw; % Wing span (ft)
        CYbetaVT; % Side force coefficient due to vertical tail
        hv; % Distance from CG to vertical tail aerodynamic center (ft)
        Clb_Gamma_factor; % Effect of Uniform Geometric Dihedral on Wing's Lateral Stability from Datcom Fig 5.1.2.1-29
    end
    
    methods
        function obj = StickFixedLateralStability(CL, Cl_beta_over_CL_Aw, KM_Lambda, KM_Gamma, Kf, Gamma, Lambda_c, theta, DeltaCl_beta_theta, Aw, deq, zw, bw, CYbetaVT, hv, Cl_beta_over_CL_C2, Clb_Gamma_factor)
            obj.CL = CL;
            obj.Cl_beta_over_CL_Aw = Cl_beta_over_CL_Aw;
            obj.KM_Lambda = KM_Lambda;
            obj.KM_Gamma = KM_Gamma;
            obj.Kf = Kf;
            obj.Gamma = Gamma;
            obj.Lambda_c = Lambda_c;
            obj.theta = theta;
            obj.DeltaCl_beta_theta = DeltaCl_beta_theta;
            obj.Aw = Aw;
            obj.deq = deq;
            obj.zw = zw;
            obj.bw = bw;
            obj.CYbetaVT = CYbetaVT;
            obj.hv = hv;
            obj.Cl_beta_over_CL_C2 = Cl_beta_over_CL_C2;
            obj.Clb_Gamma_factor = Clb_Gamma_factor;
        end

        % Calculate Cl_beta_W_Lambda
        function Cl_beta_W_Lambda = calcCl_beta_W_Lambda(obj)
            
            Cl_beta_W_Lambda = obj.CL * obj.Cl_beta_over_CL_C2 * (obj.KM_Lambda * obj.Kf);
        end
        
        % Calculate Cl_beta_W_A
        function Cl_beta_W_A = calcCl_beta_W_A(obj)
            Cl_beta_W_A = obj.CL * obj.Cl_beta_over_CL_Aw; 
        end

        % Calculate Cl_beta_W_theta
        function Cl_beta_W_theta = calcCl_beta_W_theta(obj)
            Cl_beta_W_theta = obj.theta * tand(obj.Lambda_c) * obj.DeltaCl_beta_theta;
        end
        
        % Calculate Cl_beta_W_Gamma
        function Cl_beta_W_Gamma = calcCl_beta_W_Gamma(obj)
            % Calculate the first part of the equation: Gamma * Clb_Gamma_factor * KM_Gamma
            part1 = obj.Gamma * obj.Clb_Gamma_factor * obj.KM_Gamma;
    
            % Calculate the second part of the equation: -gamma * (0.0005 * sqrt(Aw) * (deq/bw)^2)
            part2 = obj.Gamma * (0.0005 * sqrt(obj.Aw) * (obj.deq / obj.bw)^2);
    
            Cl_beta_W_Gamma = part1 - part2;
        end
        
        % Calculate Cl_beta_BW
        function Cl_beta_BW = calcCl_beta_BW(obj)
            Cl_beta_BW = ((1.2 * sqrt(obj.Aw)) / 57.3) * ((2 * obj.deq * obj.zw) / obj.bw^2);
        end
        
        % Calculate Cl_beta_VT
        function Cl_beta_VT = calcCl_beta_VT(obj)
            Cl_beta_VT = obj.CYbetaVT * (obj.hv / obj.bw);
        end
        
        % Calculate total Cl_beta
        function Cl_beta = calcCl_beta(obj)
            Cl_beta_W_Lambda = obj.calcCl_beta_W_Lambda();
            Cl_beta_W_A = obj.calcCl_beta_W_A();
            Cl_beta_W_theta = obj.calcCl_beta_W_theta();
            Cl_beta_W_Gamma = obj.calcCl_beta_W_Gamma();
            Cl_beta_BW = obj.calcCl_beta_BW();
            Cl_beta_VT = obj.calcCl_beta_VT();
            
            Cl_beta = Cl_beta_W_Lambda + Cl_beta_W_A + Cl_beta_W_theta + Cl_beta_W_Gamma + Cl_beta_BW + Cl_beta_VT;
        end
    end
end
