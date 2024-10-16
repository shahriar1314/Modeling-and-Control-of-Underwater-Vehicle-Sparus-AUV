function combinedOutput = RovModel(Thrust,PosE,VitB)

global Para




%% Attitudes in earth frame
% z=PosE(3,1);
phi     = PosE(4,1)	;
theta   = PosE(5,1)	;

%% Gravity Force

Fg = 1* [-Para.P * sin(theta) ;
        Para.P * cos(theta)*sin(phi) ;
        Para.P * cos(theta)*cos(phi) ;
        0 ;
        0 ;
        0 ];
    
% Expressed in b and computed in G
    
%% Force d'Archimède

Fa_F = [Para.B * sin(theta) ;
        -Para.B * cos(theta)*sin(phi) ;
        -Para.B * cos(theta)*cos(phi) ;
        ];
%  Expressed in b


Fa_M = S_(Para.rb-Para.rg) * Fa_F ; % Computed in G

Fa = [ Fa_F ; Fa_M ] ;
%  Expressed in b and computed in G

%% Force de Coriolis

u = VitB(1,1) ;   %Body fixed velocity surge (m*s^(-1))
v = VitB(2,1) ;   %Body fixed velocity sway (m*s^(-1))
w = VitB(3,1) ;   %Body fixed velocity heave (m*s^(-1))
p = VitB(4,1)   ;   %Body fixed velocity roll (rad*s^(-1))
q = VitB(5,1)   ;   %Body fixed velocity pitch (rad*s^(-1))
r = VitB(6,1)   ;   %Body fixed velocity yaw (rad*s^(-1))
V_ = [u;v;w]    ; % General linear velocity vector
W_ = [p;q;r]     ;  % General angular velocity vector


M11 = Para.Mg(1:3,1:3);
M12 = Para.Mg(1:3,4:6);
M21 = Para.Mg(4:6,1:3);
M22 = Para.Mg(4:6,4:6);

% General coriolis matrix :
C = [  zeros(3,3)      -S_(M11*V_ + M12*W_) ;
       -S_(M11*V_ + M12*W_)     -S_(M21 * V_ + M22 * W_) ];


% Extract diagonal elements
diagonal_elements = diag(C);

% Create a diagonal matrix
C = diag(diagonal_elements);

%C = m2c(Para.Mg,VitB);

% coriolis Force :
Fc = C * VitB;

%% Friction forces

% MAINBODY
Vit_0=VitB;
Ff_0 =  Para.S0.Kq * abs(Vit_0).*Vit_0 ;

% ANTENNA
a1 = [-0.38, 0, -0.24];
% move velocity to CG
Vit_1 = hMatrix([0,0,0], a1) * VitB;
Ff_1 =  Para.S1.Kq * abs(Vit_1).*Vit_1 ;

% THRUSTER 2
t1 = [-0.59, 0.17 , 0];
Vit_2= hMatrix([0,0,0], t1) * VitB;
% move velocity to CG
Ff_2 =  Para.S2.Kq * abs(Vit_2).*Vit_2 ;

% THRUSTER 3
t2 = [-0.59, -0.17 , 0];
Vit_3 = hMatrix([0,0,0], t2) * VitB;
% move velocity to CG
Ff_3 =  Para.S3.Kq * abs(Vit_3) .* Vit_3 ;

% USBL
s1 = [0.434, 0, -0.138];
Vit_4 = hMatrix([0,0,0], s1) * VitB;
Ff_4 = Para.S4.Kq * abs(Vit_4) .* Vit_4 ;

%% Propulsions Forces

Fp = Para.Eb * Thrust;

%% Accelearion computation :
AccG = Para.Mg\ (Ff_0 +  Ff_1 + Ff_3 + Ff_4 + Ff_2 + Fa + Fg+ Fp - Fc) ; % Mg\ = Mg^-1 computed at the gravity center of the Sparus


FORCES = [Ff_0, Ff_1, Ff_2, Ff_3, Fa, Fg, Fp, Fc];

combinedOutput = [AccG, FORCES];

