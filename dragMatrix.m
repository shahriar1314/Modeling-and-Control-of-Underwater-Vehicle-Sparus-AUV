function [D_mb, D_an, D_th, D_s1] = dragMatrix(rho, length, radius)


%% MAIN BODY



mb_Sx = pi * radius^2;
% coefficient in x direction
mb_Cd11 = 0.1; % 0.4
% coefficient in y direction
mb_Cd22 = 0.3;  % 1.9


mb_K11 = 0.5 * rho * mb_Cd11 * mb_Sx;
mb_K22 = 0.5 * rho * mb_Cd22 * 2 * radius * length;
mb_K33 = mb_K22;
mb_K55 = 1/64 * rho * length^4 * 2 * radius * mb_Cd22;
mb_K66 = mb_K55;

D_mb = -diag([mb_K11, mb_K22, mb_K33, 0, mb_K55, mb_K66]);


%% Thrusters

th_radius = 0.049; %thruster radius
th_length = 0.236;

th_Sx = pi * th_radius^2;
th_Cd11 = 0.1;  % L/D = 0.25   0.9
th_Cd22 = 0.3; %L/D = 2.62  2.5
th_Cd33 = th_Cd22; 

th_K11 = 0.5 * rho * th_Cd11 * th_Sx;
th_K22 = 0.5 * rho * th_Cd22 * 2 * th_radius * th_length;
th_K33 = th_K22;
th_K55 = 1/64 * rho * th_length^4 * 2 * th_radius * th_Cd33;
th_K66 = th_K55;

D_th = -diag([th_K11, th_K22, th_K33, 0, th_K55, th_K66]);

%% Antenna

an_x = 0.065; 
an_y = 0.03;
an_z = 0.255;

an_Sx = an_y * an_z; 
an_Cd11 = 1.27;  % using rectangular plate formula
an_Cd22 = 1.18; % L/D = 0.25   2
an_Cd33 = 2.5; 

an_K11 = 0.5 * rho * an_Cd11 * an_Sx;
an_K22 = 0.5 * rho * an_Cd22 * 2 * an_x * an_z; 
% an_K33 = 0.5 * rho * an_Cd33 * 2 * an_y * an_z; 
an_K33 = 0;
an_K55 = 1/64 * rho * an_y^4 * 2 * an_z * an_Cd33;
an_K66 = 1/64 * rho * an_z^4 * 2 * an_x * an_Cd22;

D_an = -diag([an_K11, an_K22, an_K33, 0, an_K55, an_K66]);

%% USBL

s1_a = 0.051/2; % radius
s1_b = 0.046;   % length

s1_Sx = pi * s1_a^2;
s1_Cd11 = 0.5;  % L/D = 0.45
s1_Cd22 = 0.3;
s1_Cd33 = 2.5; 

s1_K11 = 0.5 * rho * s1_Cd11 * s1_Sx;
s1_K22 = 0.5 * rho * s1_Cd22 * 2 * s1_a * s1_b;
s1_K33 = 0.5 * rho * s1_Cd33 * 2 * s1_a * s1_b;
s1_K55 = 1/64 * rho * th_length^4 * 2 * s1_a * s1_Cd33;
s1_K66 =  1/64 * rho * th_length^4 * 2 * s1_a * s1_Cd22;

D_s1= -diag([s1_K11, s1_K22, s1_K33, 0, s1_K55, s1_K66]);

end