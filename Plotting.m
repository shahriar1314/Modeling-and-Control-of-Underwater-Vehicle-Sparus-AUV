close all

tout_plot = [0; tout];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot positions 

figure(1)
subplot(2, 1, 1)
plot(tout_plot, PosE_S(:,1:3))
title("Position")
xlabel('Time (s)')
ylabel('Position (m)')
legend('x', 'y','z')
grid on

PosE_degrees = rad2deg(PosE_S(:,4:6));
subplot(2, 1, 2)
plot(tout_plot, PosE_degrees)
title("Position")
xlabel('Time (s)')
ylabel('Position (degrees)')
legend('phi','theta','psi')
grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot velocity 

figure(2)

subplot(2, 1, 1)
plot(tout_plot, VitB_S(:,1:3))
title("Velocity")
legend('u','v','w')
ylabel('Velocity (m/s)')
xlabel('Time (s)')
grid on

subplot(2, 1, 2)
VitB_degrees = rad2deg(VitB_S(:,4:6));
plot(tout_plot, VitB_degrees)
title("Velocity")
legend('p','q','r')
ylabel('Velocity (degrees/s)')
xlabel('Time (s)')
grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot acceleration 

figure(3)

subplot(2, 1, 1)
plot(tout_plot, AccB_S(:,1:3))
title("Acceleration")
legend('u dot','v dot','w dot')
xlabel('Time (s)')
ylabel('Acceleration (m^2/s)')
grid on

subplot(2, 1, 2)
AccB_degrees = rad2deg(AccB_S(:,4:6));
plot(tout_plot, AccB_degrees)
title("Acceleration")
legend('p dot','q dot','r dot')
xlabel('Time (s)')
ylabel('Acceleration (rad^2/s)')
grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot thrusters 

figure(4)
plot(tout_plot, Thrust_S)
title("Thrust")
xlabel('Time (s)')
ylabel('Thrust (N)')
legend('F1','F2','F3')
grid on


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot forces 

% Number of forces
numForces = size(FORCES, 2);

% Set the figure size
figure('Position', [100, 100, 800, 800]);

% Define force names
force_names = {'Drag force (mainbody)', 'Drag force (antenna)', 'Drag force (thruster 1)', 'Drag force (thruster 2)', 'Archimede Force', 'Gravity force', 'Thrusters', 'Coriolis Force'};

% Plot each force separately
for i = 1:length(force_names)
    subplot(4, 2, i);  % Adjust to 4 rows and 2 columns
    
    % Calculate indices for the current force
    indices = (i-1)*6 + 1:i*6;
    
    % Plot the corresponding columns
    plot(tout_plot, FORCES(:, indices));
    
    title(force_names{i});
    xlabel('Time (s)');
    ylabel('Force Value (N)');
    legend('x','y','z','phi','theta','psi','Location', 'BestOutside');
end
