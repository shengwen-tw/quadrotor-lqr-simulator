function quadrotor_sim
ITERATION_TIMES = 10000;

math = se3_math;

uav_dynamics = dynamics;        %create uav dynamics object
uav_dynamics.dt = 0.001;        %set iteration period [sec]
uav_dynamics.mass = 1;          %set uav mass [kg]
uav_dynamics.a = [0; 0; 0];     %acceleration of uav [m/s^2], effected by applied force
uav_dynamics.v = [0; 0; 0];     %initial velocity of uav [m/s]
uav_dynamics.x = [0.5; 0; 0];     %initial position of uav [m]
uav_dynamics.W = [0; 0; 0];     %initial angular velocity of uav
uav_dynamics.W_dot = [0; 0; 0]; %angular acceleration of uav, effected by applied moment
uav_dynamics.f = [0; 0; 0];     %force generated by controller
uav_dynamics.M = [0; 0; 0];     %moment generated by controller

%set initial attitude (DCM)
init_attitude(1) = deg2rad(0);  %roll
init_attitude(2) = deg2rad(0);  %pitch
init_attitude(3) = deg2rad(0);  %yaw
uav_dynamics.R = math.euler_to_dcm(init_attitude(1), init_attitude(2), init_attitude(3));

uav_dynamics.J = [0.01466 0 0;  %inertia matrix of uav
                  0 0.01466 0;
                  0 0 0.02848];

quad_sim_greeting(uav_dynamics, ITERATION_TIMES, init_attitude);

%lqr parameters
Q = zeros(12, 12);
Q(1, 1) = 20;     %roll
Q(2, 2) = 20;     %pitch
Q(3, 3) = 20;     %yaw
Q(4, 4) = 5;      %roll rate
Q(5, 5) = 5;      %pitch rate
Q(6, 6) = 5;      %yaw rate
Q(7, 7) = 200;    %vx
Q(8, 8) = 200;    %vy
Q(9, 9) = 200;    %vz
Q(10, 10) = 5000; %x
Q(11, 11) = 5000; %y
Q(12, 12) = 1000; %z

R = zeros(4, 4);
R(1, 1) = 0.1;    %f
R(2, 2) = 1;      %wx
R(3, 3) = 1;      %wy
R(4, 4) = 1;      %wz

m = uav_dynamics.mass;
g = 9.8;
Ix = uav_dynamics.J(1, 1);
Iy = uav_dynamics.J(2, 2);
Iz = uav_dynamics.J(3, 3);

%construct A matrix (small angle approximation, not used)
A = [0  0 0 1 0 0 0 0 0 0 0 0;
     0  0 0 0 1 0 0 0 0 0 0 0;
     0  0 0 0 0 1 0 0 0 0 0 0;
     0  0 0 0 0 0 0 0 0 0 0 0;
     0  0 0 0 0 0 0 0 0 0 0 0;
     0  0 0 0 0 0 0 0 0 0 0 0;
     0 -g 0 0 0 0 0 0 0 0 0 0;
     g  0 0 0 0 0 0 0 0 0 0 0;
     0  0 0 0 0 0 0 0 0 0 0 0;
     0  0 0 0 0 0 1 0 0 0 0 0;
     0  0 0 0 0 0 0 1 0 0 0 0;
     0  0 0 0 0 0 0 0 1 0 0 0];

%construct B matrix
B = [ 0   0   0   0;
      0   0   0   0;
      0   0   0   0;
      0  1/Ix 0   0;
      0   0  1/Iy 0;
      0   0   0  1/Iz;
      0   0   0   0;
      0   0   0   0;
    -1/m  0   0   0;
      0   0   0   0;
      0   0   0   0;
      0   0   0   0];

%solve CARE
C = eye(12);
H = transpose(C)*Q*C;
%[X, L, G] = care(A, B, H, R);
X = care_sda(A, B, H, R);
K = inv(R) * transpose(B) * X;

%controller setpoints
xd = zeros(3, ITERATION_TIMES);
vd = zeros(3, ITERATION_TIMES);
yaw_d = zeros(1, ITERATION_TIMES);

%%%%%%%%%%%%%%%%%%%%%
%   path planning   %
%%%%%%%%%%%%%%%%%%%%%
% cirular trajectory
radius = 0.5;         %[m]
circum_rate = 0.25;   %[hz], times of finished a circular trajectory per second
climb_rate = -0.05;
yaw_rate = 0.05;      %[hz], times of full rotation around z axis per second
for i = 1: ITERATION_TIMES
    %plan heading
    if i == 1
        yaw_d(1) = 0;
    else
        yaw_d(i) = yaw_d(i - 1) + (yaw_rate * uav_dynamics.dt * 2 * pi);
    end
    if yaw_d(i) > pi   %bound yaw angle between +-180 degree
        yaw_d(i) = yaw_d(i) - (2 * pi);
    end
    
    %plan position
    xd(1, i) = radius * cos(circum_rate * uav_dynamics.dt * i * pi);
    xd(2, i) = radius * sin(circum_rate * uav_dynamics.dt * i * pi);
    
    xd(3, i) = i * uav_dynamics.dt * climb_rate;
    if(xd(3, i) <= -1)
        xd(3, i) = -1;
    end
    
    %plan velocity
    vd(1, i) = radius * -sin(circum_rate * uav_dynamics.dt * i * pi);
    vd(2, i) = radius * cos(circum_rate * uav_dynamics.dt * i * pi);
    vd(3, i) = climb_rate;
end

%plot datas
time_arr = zeros(1, ITERATION_TIMES);
speed_inc_arr = zeros(1, ITERATION_TIMES);
sda_time_arr = zeros(1, ITERATION_TIMES);

matlab_x_norm_arr = zeros(1, ITERATION_TIMES);
sda_x_norm_arr = zeros(1, ITERATION_TIMES);

matlab_time_arr = zeros(1, ITERATION_TIMES);
vel_arr = zeros(3, ITERATION_TIMES);
R_arr = zeros(3, 3, ITERATION_TIMES);
euler_arr = zeros(3, ITERATION_TIMES);
pos_arr = zeros(3, ITERATION_TIMES);
W_arr = zeros(3, ITERATION_TIMES);
M_arr = zeros(3, ITERATION_TIMES);

G = B*inv(R)*transpose(B);
for i = 1: ITERATION_TIMES
    %disp(i);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Update System Dynamics %
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    uav_dynamics = update(uav_dynamics);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Quadrotor LQR Control %
    %%%%%%%%%%%%%%%%%%%%%%%%%
    
    eulers = math.dcm_to_euler(uav_dynamics.R); %get euler angles from R matrix
    v_b = uav_dynamics.R * uav_dynamics.v;      %get body frame velocity
    
    p = uav_dynamics.W(1);
    q = uav_dynamics.W(2);
    r = uav_dynamics.W(3);
    u = v_b(1);
    v = v_b(2);
    w = v_b(3);
    
    %construct A matrix
    s_phi = sin(eulers(1));
    c_phi = cos(eulers(1));
    s_theta = sin(eulers(2));
    c_theta = cos(eulers(2));
    s_psi = sin(eulers(3));
    c_psi = cos(eulers(3));
    t_theta = tan(eulers(2));
    sec_theta = sec(eulers(2));
    
    a1 = [-r*s_phi*t_theta + q*c_phi*t_theta ...
          r*(c_phi*sec_theta^2 + q*s_phi*sec_theta^2) ...
          0 1 s_phi*t_theta c_phi*t_theta 0 0 0 0 0 0];
    a2 = [(-q*s_phi - r*c_phi) 0 0 0 c_phi -s_phi 0 0 0 0 0 0];
    a3 = [-r*s_phi/c_theta + q*c_phi/c_theta ...
          r*c_phi*sec_theta*t_theta + q*s_phi*sec_theta*t_theta ...
          0 0 s_phi/c_theta c_phi/c_theta 0 0 0 0 0 0];
    a4 = [0 0 0 0 (Iy-Iz)/Ix*r (Iy-Iz)/Ix*q 0 0 0 0 0 0];
    a5 = [0 0 0 (Iz-Ix)/Iy*r 0 (Iz-Ix)/Iy*p 0 0 0 0 0 0];
    a6 = [0 0 0 (Ix-Iy)/Iz*q (Ix-Iy)/Iz*p 0 0 0 0 0 0 0];
    a7 = [0 -g*c_theta 0 0 -w v 0 r -q 0 0 0];
    a8 = [g*c_phi*c_theta -g*s_phi*s_theta 0 w 0 -u -r 0 p 0 0 0];
    a9 = [-g*c_theta*s_phi -g*s_theta*c_phi 0 -v u 0 q -p 0 0 0 0];
    a10 = [w*(c_phi*s_psi - s_phi*c_psi*s_theta) + v*(s_phi*s_psi + c_psi*c_phi*s_theta) ...
           w*(c_phi*c_psi*c_theta) + v*(c_psi*s_phi*c_theta) - u*(c_psi*s_theta) ...
           w*(s_phi*c_psi - c_phi*s_psi*s_theta) - v*(c_phi*c_psi - c_phi*c_psi*s_theta) + u*(c_theta*c_psi) ...
           0 0 0 c_psi*c_theta (-c_phi*s_psi + c_psi*s_phi*s_theta) (s_phi*s_psi + c_phi*c_psi*s_theta) 0 0 0];
    a11 = [v*(-s_phi*c_psi + c_phi*s_psi*s_theta) - w*(c_psi*c_phi + s_phi*s_psi*s_theta) ...
           v*(s_phi*s_psi*c_theta) + w*(c_phi*s_psi*c_theta) - u*(s_theta*s_psi) ...
           v*(-c_phi*s_psi + s_phi*c_psi*s_theta) + w*(s_psi*s_phi + c_phi*c_psi*s_theta) + u*(c_theta*c_psi) ...
           0 0 0 c_theta*s_psi (c_phi*c_psi + s_phi*s_psi*s_theta) (-c_psi*s_phi + c_phi*s_psi*s_theta) 0 0 0];
    a12 = [-w*s_phi*c_theta + v*c_theta*c_phi ...
           -w*c_phi*s_theta - u*c_theta - v*s_theta*s_phi ...
           0 0 0 0 -s_theta c_theta*s_phi c_phi*c_theta 0 0 0];
    A = [a1; a2; a3; a4; a5; a6; a7; a8; a9; a10; a11; a12];

    At = transpose(A);
    
    tstart = tic();
    X = care_sda(A, B, H, R);
    tend = tic();
    sda_x_norm = norm(At*X + X*A - X*G*X + H);
    sda_time = tend - tstart;
    
    tstart = tic();
    [X, L, G_dummy] = care(A, B, H, R);
    tend = tic();
    matlab_x_norm = norm(At*X + X*A - X*G*X + H);
    matlab_time = tend - tstart;

    speed_inc = matlab_time / sda_time;
    
    K = inv(R) * transpose(B) * X;
    
    p = uav_dynamics.W(1);
    q = uav_dynamics.W(2);
    r = uav_dynamics.W(3);
    u = v_b(1);
    v = v_b(2);
    w = v_b(3);
    
    %construct state vector
    x = [eulers(1);
         eulers(2);
         eulers(3);
         uav_dynamics.W(1);
         uav_dynamics.W(2);
         uav_dynamics.W(3);
         v_b(1);
         v_b(2);
         v_b(3);
         uav_dynamics.x(1);
         uav_dynamics.x(2);
         uav_dynamics.x(3)];
    
    %construct desired setpoint vector
    x0 = [deg2rad(0); %desired roll
          deg2rad(0); %desired pitch
          deg2rad(0); %desired yaw
          0;          %desired roll
          0;          %desired pitch
          0;          %desired yaw
          vd(1, i);   %desired x velocity
          vd(2, i);   %desired y velocity
          vd(3, i);   %desired z velocity
          xd(1, i);   %desired x position
          xd(2, i);   %desired y position
          xd(3, i)];  %desired z position
    
    %calculate feedforward control
    gravity_ff = dot(uav_dynamics.mass .* g .* [0; 0; 1], uav_dynamics.R * [0; 0; 1]);
    u_ff = [gravity_ff; 0; 0; 0];
    
    %calculate feedback control
    u_fb = -K * [x - x0];
    
    %obtain complete control input
    u = u_ff + u_fb;
    
    lqr_f = uav_dynamics.R * [0; 0; u(1)];
    lqr_M = [u(2); u(3); u(4)];
    
    %feed control to uav dynamics
    uav_dynamics.f = lqr_f;
    uav_dynamics.M = lqr_M;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Record datas for plotting %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    time_arr(i) = i * uav_dynamics.dt;
    sda_time_arr(i) = sda_time;
    matlab_time_arr(i) = matlab_time;
    matlab_x_norm_arr(i) = matlab_x_norm;
    sda_x_norm_arr(i) = sda_x_norm;
    speed_inc_arr(i) = speed_inc;
    sda_time_arr(i) = sda_time;
    vel_arr(:, i) = uav_dynamics.v;
    pos_arr(:, i) = uav_dynamics.x;
    R_arr(:, :, i) = uav_dynamics.R;
    euler_arr(:, i) = rad2deg(math.dcm_to_euler(uav_dynamics.R));
    W_arr(:, i) = rad2deg(uav_dynamics.W);
    M_arr(:, i) = uav_dynamics.M;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Animate the simulation result %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rigidbody_visualize([5; 5; 5], pos_arr, R_arr, ITERATION_TIMES, uav_dynamics.dt, 30);

%%%%%%%%%%%%%%%%%%%%%%%%%%
%          Plot          %
%%%%%%%%%%%%%%%%%%%%%%%%%%

%attitude (euler angles)
figure('Name', 'attitude (euler angles)');
subplot (3, 1, 1);
plot(time_arr, euler_arr(1, :));
xlabel('time [s]');
ylabel('roll [deg]');
subplot (3, 1, 2);
plot(time_arr, euler_arr(2, :));
xlabel('time [s]');
ylabel('pitch [deg]');
subplot (3, 1, 3);
plot(time_arr, euler_arr(3, :));
xlabel('time [s]');
ylabel('yaw [deg]');

%angular velocity
figure('Name', 'Angular velocity');
subplot (3, 1, 1);
plot(time_arr, W_arr(1, :));
xlabel('time [s]');
ylabel('x [deg/s]');
subplot (3, 1, 2);
plot(time_arr, W_arr(2, :));
xlabel('time [s]');
ylabel('y [deg/s]');
subplot (3, 1, 3);
plot(time_arr, W_arr(3, :));
xlabel('time [s]');
ylabel('z [deg/s]');

%velocity
figure('Name', 'velocity (NED frame)');
subplot (3, 1, 1);
plot(time_arr, vel_arr(1, :), time_arr, vd(1, :));
xlabel('time [s]');
ylabel('x [m/s]');
subplot (3, 1, 2);
plot(time_arr, vel_arr(2, :), time_arr, vd(2, :));
xlabel('time [s]');
ylabel('y [m/s]');
subplot (3, 1, 3);
plot(time_arr, -vel_arr(3, :), time_arr, -vd(3, :));
xlabel('time [s]');
ylabel('-z [m/s]');

%position
figure('Name', 'position (NED frame)');
subplot (3, 1, 1);
plot(time_arr, pos_arr(1, :), time_arr, xd(1, :));
xlabel('time [s]');
ylabel('x [m]');
subplot (3, 1, 2);
plot(time_arr, pos_arr(2, :), time_arr, xd(2, :));
xlabel('time [s]');
ylabel('y [m]');
subplot (3, 1, 3);
plot(time_arr, -pos_arr(3, :), time_arr, -xd(3, :));
xlabel('time [s]');
ylabel('-z [m]');

%position
figure('Name', 'time cost of CARE solvers');
title('time cost');
plot(time_arr, sda_time_arr, time_arr, matlab_time_arr);
xlabel('time [s]');
ylabel('cost [s]');

%position
figure('Name', 'SDA / MATLAB CARE');
plot(time_arr, speed_inc_arr);

disp(mean(speed_inc_arr));

%position
figure('Name', 'precision of CARE solvers');
title('precision (norm of CARE)');
sda_x_norm_arr(1:5) = [];
matlab_x_norm_arr(1:5) = [];
time_arr(1:5) = [];
plot(time_arr, sda_x_norm_arr, time_arr, matlab_x_norm_arr);
xlabel('time [s]');
ylabel('CARE norm');

disp("Press any key to leave");
pause;
close all;
end

function quad_sim_greeting(dynamics, iteration_times, init_attitude)
roll = rad2deg(init_attitude(1));
pitch = rad2deg(init_attitude(2));
yaw = rad2deg(init_attitude(3));
disp(sprintf('Quadrotor simulation (%d iterations, dt = %dsec)', iteration_times, dynamics.dt));
disp(sprintf('Initial position: (%f, %f, %f)', dynamics.x(1), dynamics.x(2), dynamics.x(3)));
disp(sprintf('Initial attitude (euler angle): (%f, %f, %f)', roll, pitch, yaw));
disp('Start simulation...');
end
