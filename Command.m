function [Out] = Command(PosE, VitB, AccB)

% PID controller gains for heave position, surge position, and surge velocity
Kp_heave = 1; % Proportional gain for heave position
Ki_heave = 0.1; % Integral gain for heave position
Kd_heave = 0.01; % Derivative gain for heave position

Kp_surge_pos = 1; % Proportional gain for surge position
Ki_surge_pos = 0.1; % Integral gain for surge position
Kd_surge_pos = 0.01; % Derivative gain for surge position

Kp_surge_vel = 1; % Proportional gain for surge velocity
Ki_surge_vel = 0.1; % Integral gain for surge velocity
Kd_surge_vel = 0.01; % Derivative gain for surge velocity

% Desired positions and velocities (you can modify these as needed)
Desired_heave_pos = 0;
Desired_surge_pos = 0;
Desired_surge_vel = 0;

% Error terms
Error_heave_pos = Desired_heave_pos - PosE(3);
Error_surge_pos = Desired_surge_pos - PosE(1);
Error_surge_vel = Desired_surge_vel - VitB(1);

% PID control outputs
PID_heave_pos = Kp_heave * Error_heave_pos + Ki_heave * sum(Error_heave_pos) + Kd_heave * (Error_heave_pos - Out(3));
PID_surge_pos = Kp_surge_pos * Error_surge_pos + Ki_surge_pos * sum(Error_surge_pos) + Kd_surge_pos * (Error_surge_pos - Out(1));
PID_surge_vel = Kp_surge_vel * Error_surge_vel + Ki_surge_vel * sum(Error_surge_vel) + Kd_surge_vel * (Error_surge_vel - VitB(1));

% Thrust commands
Thrust = [PID_surge_pos 0 PID_heave_pos];

% Output
Out = Thrust;
end

