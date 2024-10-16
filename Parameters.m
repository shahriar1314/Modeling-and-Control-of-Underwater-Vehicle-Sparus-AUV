function Para=Parameters()
global Para

%% Initial Speed and position in Earth-fixed frame

Para.ICPos = [0 0 2 0 0 0];
Para.ICSpeed = [0 0 0 0 0 0] ;

%% General parameters
Para.rho_water = 1000 ;                     % Masse volumique de l'eau (kg/m^3)
Para.R = 0.115 ;                             % Sparus Radius (m)
Para.L = 1.6;  	                            % Sparus length (m)
Para.m = 52 ; 	                            % Sparus mass (kg)
Para.mb = 52.1;                           	% Sparus buoyancy mass  (kg) 
Para.g = 9.81 ;                             % Earth Gravity (m*s^(-2))
Para.P = Para.m * Para.g;	                % Sparus weight (N)
Para.B = Para.mb * Para.g;	                % Buoyancy (N)

%% Center of gravity and Buoyancy position in body-fixed frame

Para.xg = 0 ;    %x-positon of Center of gravity
Para.yg = 0 ;    %y-positon of Center of gravity
Para.zg = 0 ;    %z-positon of Center of gravity

Para.rg = [Para.xg Para.yg Para.zg]' ;


Para.xb = 0      ;    % x-positon of Center of Buoyancy
Para.yb = 0      ;    % y-positon of Center of Buoyancy
Para.zb = -0.02  ;    % z-positon of Center of Buoyancy

Para.rb = [Para.xb Para.yb Para.zb]' ;

%% Body positions


%Thruster positions in body-fixed frame

Para.d1x = 0        ; 
Para.d1y = 0        ;
Para.d1z = 0.08     ;
Para.d2x = -0.59    ; 
Para.d2y = 0.17     ;
Para.d2z = 0        ;
Para.d3x = -0.59    ;
Para.d3y = -0.17    ;
Para.d3z = 0        ;

% Main Body S0;
Para.S0.r=[0,0,0]'; % Position (m)
% First Body S1: antenna;
Para.S1.r= [-0.38, 0,  -0.24]'; % Position (m)
% Second Body S2: thruster 2;
Para.S2.r=[Para.d2x, Para.d2y, Para.d2z]'; % Position (m)
% Third Body S3: thruster 3;
Para.S3.r=[Para.d3x, Para.d3y, Para.d3z]'; % Position (m)

%% Body Mass matrices


Para.S0.Mb = [
    52, 0, 0, 0, -0.1, 0;
    0, 52, 0, 0.1, 0, -1.3;
    0, 0, 52, 0, 1.3, 0;
    0, 0.1, 0, 0.5, 0, 0;
    -0.1, 0, 1.3, 0, 9.4, 0;
    0, -1.3, 0, 0, 0, 9.5
];


%% Body added Mass matrices


[mA_mainbody, a_Ma_CG, t1_Ma_CG, t2_Ma_CG, s1_Ma_CG] = AddedMass_CB(Para.rho_water, Para.L, Para.R);

% Para.S0.Ma = diag(diag(mA_mainbody));
% Para.S1.Ma = diag(diag(a_Ma_CG));
% Para.S2.Ma = diag(diag(t1_Ma_CG));
% Para.S3.Ma = diag(diag(t2_Ma_CG));


Para.S0.Ma = mA_mainbody;
Para.S1.Ma = a_Ma_CG;
Para.S2.Ma = t1_Ma_CG;
Para.S3.Ma = t2_Ma_CG;

%% Generalized mass matrix

Para.S0.Mg = Para.S0.Mb + Para.S0.Ma ; 
Para.S1.Mg = Para.S1.Ma ;
Para.S2.Mg = Para.S2.Ma ;
Para.S3.Mg = Para.S3.Ma ;
Para.S4.Mg = s1_Ma_CG;


Para.Mg = Para.S0.Mg + Para.S1.Mg + Para.S2.Mg + Para.S3.Mg + Para.S4.Mg

%% Generalized coriolis matrix

% Computed in RovModel.m

%% Friction matrices

[Para.S0.Kq, Para.S1.Kq, Para.S2.Kq, Para.S4.Kq] = dragMatrix(Para.rho_water, Para.L, Para.R);

Para.S3.Kq = Para.S2.Kq;

%% Thruster modelling

% Thruster gains
Para.kt1 = 55;   % Forward
Para.kt2 = 71.5; % Forward
Para.kt3 = 71.5; % Forward


%Thruster gain vectors

Para.Kt=[Para.kt1;Para.kt2;Para.kt3];

%Thruster time constants

Para.Tau1 = 0.4 ;
Para.Tau2 = 0.8 ;
Para.Tau3 = 0.8 ;

%Thruster time constant vectors

Para.Tau = [Para.Tau1;Para.Tau2;Para.Tau3] ;


% Mapping of thruster

Para.rt1 = [Para.d1x, Para.d1y, Para.d1z]' ;
Para.rt2 = [Para.d2x, Para.d2y, Para.d2z]' ;
Para.rt3 = [Para.d3x, Para.d3y, Para.d3z]' ;


Para.rt = [Para.rt1 Para.rt2 Para.rt3] ;


Para.Eb_F = [0 1 1;
    0 0 0;
    1 0 0];
    
Para.Eb_M = [Para.d1y 0 0;
-Para.d1x Para.d2z Para.d3z;
0 -Para.d2y -Para.d3y] ;

Para.Eb = [ Para.Eb_F ; Para.Eb_M];


% Inverse Mapping of thruster
Para.Ebinv = pinv(Para.Eb);

end





 
           

