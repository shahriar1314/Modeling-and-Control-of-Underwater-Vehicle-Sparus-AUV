% ADDED MASS
function [mA_mainbody, a_Ma_CB, t1_Ma_CB, t2_Ma_CB, s1_Ma_CG] = AddedMass_CB(rho, length, radius)


%% Main body

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% LAMB'S K FACTOR %%%%%%%%%%

% added mass of the main body in the x direction

m_df = 4/3 * rho * pi * length/2 * radius^2;
ecc = sqrt(1-radius^2/((length/2)^2));
alpha_0 = 2 * (1 - ecc^2)/ecc^3 * (1/2 * log((1 + ecc)/(1 - ecc)) - ecc);
k1 = alpha_0/(2-alpha_0);

ma11 = k1 * m_df;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% SLENDER BODY THEORY %%%%%%%%%%


%%%%%% 1 - FIRST SECTION: CYLINDER %%%%%%

b1_Ca = 1;
b1_Ar = pi * (radius^2);
b1_l1 = -0.629;
b1_l2 = 0.730;

b1_a22 = rho * b1_Ca * b1_Ar;
b1_a33 = b1_a22;
b1_a44 = 0;

b1_Ma = x_slenderMatrix(b1_a22, b1_a33, b1_a44, b1_l1, b1_l2);


%%%%%% 3 - THIRD SECTION: AFT %%%%%%

b3_Ca = 1;
b3_l1 = -0.755;
b3_l2 = -0.629;

 % parameters of the line to integrate the radius
m = radius/(b3_l1-b3_l2);
q = - m * b3_l1;

b3_a22 = @(x) rho * b3_Ca * pi * (m * x + q).^2;
b3_a33 = b3_a22;
b3_a44 = 0;

b3_Ma = x_slenderMatrix(b3_a22, b3_a33, b3_a44, b3_l1,b3_l2);

%%%%%% 4 - FOURTH SECTION: FORE %%%%%%

% b4_Ca = 1;
b4_l1 = 0.730;
b4_l2 = 0.845;

 % parameters of the circle to integrate the radius
b4_a22 = @(x) rho * pi * (-x.^2 - 0.52 + 1.46 * x);
b4_a33 = b4_a22;
b4_a44 = 0;

b4_mA = x_slenderMatrix(b4_a22, b4_a33, b4_a44, b4_l1, b4_l2);

%%%%%% Adding up %%%%%%

mA_mainbody = b1_Ma + b3_Ma + b4_mA;
mA_mainbody(1,1) = ma11;

% mA_mainbody

%% Thrusters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% 3D ADDED MASS %%%%%%%%%%%%

% calculate with slender body theory + 3D coefficient

t1_len = 0.23;
t1_r = 0.09/2;
t1_Ca = 0.62;
t1_Ar = pi * (t1_r^2);

t1_a22 = rho * t1_Ca * t1_Ar;
t1_a33 = t1_a22;
t1_a44 = 0;

t1_Ma = x_slenderMatrix(t1_a22, t1_a33, t1_a44, -t1_len/2, t1_len/2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% LAMB'S K-FACTOR %%%%%%%%%%%

% added mass of the thruster in the x direction

t1_a = 0.194;
t1_b = 0.069;

m_df2 = 4/3 * rho * pi * t1_a/2 * t1_b^2;
ecc2 = sqrt(1-t1_b^2/((t1_a/2)^2));
alpha_02 = 2 * (1 - ecc2^2)/ecc2^3 * (1/2 * log((1 + ecc2)/(1 - ecc2)) - ecc2);
k1_2 = alpha_02/(2-alpha_02);

t1_ma11 = k1_2 * m_df2;

t1_Ma(1,1) = t1_ma11;
t2_Ma = t1_Ma;

d2x = -0.59; 
d2y = 0.17;
d2z = 0;
d3x = -0.59;
d3y = -0.17;
d3z = 0;

t1 = [d2x, d2y, d2z];
t2 = [d3x, d3y, d3z];

t1_Ma_CB = traslatedMatrix(t1_Ma, t1, [0, 0, -0.02]);
t2_Ma_CB = traslatedMatrix(t2_Ma, t2, [0, 0, -0.02]);

% t1_Ma_CG = zeros(6);
% t2_Ma_CG = zeros(6);
%% ANTENNA 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% SLENDER BODY THEORY %%%%%%%%%%%%

% coords of the center of the antenna
a1 = [-0.38, 0,  -0.24];
% a1 =  [0, 0, 0];
% length of antenna
a1_l = 0.255;

% other variables for antenna

a1_Ca = 1.8;
a2_Ca = 1.36;
a6_Ca = 0.15;

a_b = 0.06/2; % antenna length along x direction
a_a = 0.023/2;  % antenna length along y direction

a_ma11 = rho * a1_Ca * pi * (a_a)^2;
a_ma22 = rho * a2_Ca * pi * (a_b)^2;
a_ma66 = rho * a6_Ca * pi * (a_b)^4;

% antenna's added mass matrix in antenna cg
a_Ma = z_slenderMatrix(a_ma11, a_ma22, a_ma66, -a1_l/2, a1_l/2);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% LAMB'S K-FACTOR %%%%%%%%%%%

% added mass of the antenna in the z direction

a1_a = 0.25/2;
a1_b = 0.03;

m_df3 = 4/3 * rho * pi * a1_a/2 * a1_b^2;
ecc3 = sqrt(1-a1_b^2/((a1_a/2)^2));
alpha_03 = 2 * (1 - ecc3^2)/ecc3^3 * (1/2 * log((1 + ecc3)/(1 - ecc3)) - ecc3);
k1_3 = alpha_03/(2-alpha_03);

a1_ma33 = k1_3 * m_df3;

a_Ma(3,3) = a1_ma33;


% antenna's added mass matrix in co
a_Ma_CB = traslatedMatrix(a_Ma, a1, [0, 0, -0.02])


%% SENSORS

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% 3D ADDED MASS %%%%%%%%%%%%

s1 = [0.434, 0, -0.138];
s1_a = 0.051/2; % radius
s1_b = 0.046;   % length
s1_ratio = 0.046/(s1_a*2);
x = [1.2 2.5 5 9];
v = [0.62 0.78 0.90 0.96];
plot(x,v);
s1_Ca = interp1(x, v, s1_ratio, 'spline');

s1_ma11 = pi * s1_a^2 * s1_b * s1_Ca;    % multiply by 3D coefficient
s1_ma22 = s1_ma11;
s1_Ma = z_slenderMatrix(s1_ma11, s1_ma22, 0, -s1_b/2, s1_b/2);

s1_Ma_CB = traslatedMatrix(s1_Ma, s1, [0, 0, -0.02]);
s1_Ma_CG = traslatedMatrix(s1_Ma_CB, [0, 0, -0.02], [0, 0, 0]);


% we can now definetely neglect the other tiny sensor

%% Total

% add all the added mass matrices

mA_CB = mA_mainbody + t1_Ma_CB + t2_Ma_CB + a_Ma_CB

% translate to the center of buoyancy

mA = traslatedMatrix(mA_CB, [0, 0,  -0.02], [0, 0, 0])



function Ma = x_slenderMatrix(a22, a33, a44, l1, l2)
    if ~isa(a22, 'function_handle')
        a22 = @(x) a22;  % Create a constant function handle that returns the original expression
    end
    if ~isa(a33, 'function_handle')
    a33 = @(x) a33;  % Create a constant function handle that returns the original expression
    end
    if ~isa(a44, 'function_handle')
        a44 = @(x) a44;  % Create a constant function handle that returns the original expression
    end

    % Calculate the values
    
    ma22 = integral(a22, l1, l2, 'ArrayValued', true);
    ma33 = integral(a33, l1, l2, 'ArrayValued', true);
    ma44 = integral(a44, l1, l2, 'ArrayValued', true);
    
    in_a35 = @(x) x .* a33(x);
    in_a62 = @(x) x .* a22(x);
    in_a55 = @(x) (x.^2) .* a33(x);
    in_a66 = @(x) (x.^2) .* a22(x);

    ma55 = integral(in_a55, l1, l2);
    ma66 = integral(in_a66, l1, l2);
    ma35 = -integral(in_a35, l1, l2);
    ma62 = integral(in_a62, l1, l2);
    
    ma53 = ma35;
    ma26 = ma62;
    
    
    % Create the matrix Ma
    Ma = createMatrix(0, ma22, ma33, ma44, ma55, ma66, 0, 0, ma26, ma35, 0, 0, ma53,  ma62);
end


function Ma = z_slenderMatrix(a11, a22, a66, l1, l2)
    if ~isa(a11, 'function_handle')
        a11 = @(z) a11;  % Create a constant function handle that returns the original expression
    end
    if ~isa(a22, 'function_handle')
        a22 = @(z) a22;  % Create a constant function handle that returns the original expression
    end
    if ~isa(a66, 'function_handle')
        a66 = @(z) a66;  % Create a constant function handle that returns the original expression
    end

    % Calculate the values

    ma11 = integral(a11, l1, l2, 'ArrayValued', true);
    ma22 = integral(a22, l1, l2, 'ArrayValued', true);
    ma66 = integral(a66, l1, l2, 'ArrayValued', true);
    
    in_a42 = @(z) z .* a22(z);
    in_a51 = @(z) z .* a11(z);
    in_a44 = @(z) (z.^2) .* a22(z);
    in_a55 = @(z) (z.^2) .* a11(z);

    ma42 = - integral(in_a42, l1, l2);
    ma51 = integral(in_a51, l1, l2); 
    ma44 = integral(in_a44, l1, l2);
    ma55 = integral(in_a55, l1, l2);
    
    ma15 = ma51;
    ma24 = ma42;
    
    
    % Create the matrix Ma
    Ma = createMatrix(ma11, ma22, 0, ma44, ma55, ma66, ma15, ma24, 0, 0, ma42, ma51, 0, 0);
end


function Ma = createMatrix(ma11, ma22, ma33, ma44, ma55, ma66, ma15, ma24, ma26, ma35, ma42, ma51, ma53, ma62)
    % Create the matrix Ma
    Ma = [ma11 0 0 0 ma15 0;
          0 ma22 0 ma24 0 ma26;
          0 0 ma33 0 ma35 0;
          0 ma42 0 ma44 0 0;
          ma51 0 ma53 0 ma55 0;
          0 ma62 0 0 0 ma66];
end


end