function quadrotor_sim
	ITERATION_TIMES = 10000;

	math = se3_math;

	uav_dynamics = dynamics;        %create uav dynamics object
	uav_dynamics.dt = 0.001;        %set iteration period [sec]
	uav_dynamics.mass = 1;          %set uav mass [kg]
	uav_dynamics.a = [0; 0; 0];     %acceleration of uav [m/s^2], effected by applied force
	uav_dynamics.v = [0; 0; 0];     %initial velocity of uav [m/s]
	uav_dynamics.x = [0; 0; 0];    %initial position of uav [m]
	uav_dynamics.W = [0; 0; 0];     %initial angular velocity of uav
	uav_dynamics.W_dot = [0; 0; 0]; %angular acceleration of uav, effected by applied moment
	uav_dynamics.f = [0; 0; 0];     %force generated by controller
	uav_dynamics.M = [0; 0; 0];     %moment generated by controller
    
	%set initial attitude (DCM)
	init_attitude(1) = deg2rad(0); %roll
	init_attitude(2) = deg2rad(0); %pitch
	init_attitude(3) = deg2rad(0); %yaw
	uav_dynamics.R = math.euler_to_dcm(init_attitude(1), init_attitude(2), init_attitude(3));

	uav_dynamics.J = [0.01466 0 0;  %inertia matrix of uav
			          0 0.01466 0;
			          0 0 0.02848];
          
	quad_sim_greeting(uav_dynamics, ITERATION_TIMES, init_attitude);

	%lqr parameters
    Q = zeros(12, 12);
    Q(1, 1) = 5; %roll
    Q(2, 2) = 5; %pitch
    Q(3, 3) = 1; %yaw
    Q(4, 4) = 5; %roll rate
    Q(5, 5) = 5; %pitch rate
    Q(6, 6) = 5; %yaw rate
    Q(7, 7) = 1; %vx
    Q(8, 8) = 1; %vy
    Q(9, 9) = 1; %vz
    Q(10, 10) = 5; %x
    Q(11, 11) = 5; %y
    Q(12, 12) = 70; %z
    
    R = zeros(4, 4);
    R(1, 1) = 0.1; %f
    R(2, 2) = 1; %wx
    R(3, 3) = 1; %wy
    R(4, 4) = 1; %wz
    
    %bodyrate controller
    kW = [2; 2; 2];
    
	%controller setpoints	
	xd = zeros(3, ITERATION_TIMES);
	vd = zeros(3, ITERATION_TIMES);
	a_d = [0; 0; 0];
	yaw_d = zeros(1, ITERATION_TIMES);
	Wd = [0; 0; 0];
	W_dot_d = [0; 0; 0];

	%%%%%%%%%%%%%%%%%%%%%
	%   path planning   %
	%%%%%%%%%%%%%%%%%%%%%
	% cirular motion
	radius = 3;        %[m]
	circum_rate = 0.25; %[hz], times of finished a circular trajectory per second
	yaw_rate = 0.05;    %[hz], times of full rotation around z axis per second
	for i = 1: ITERATION_TIMES
		%plan heading
		if i == 1
			yaw_d(1) = 0;
		else
			yaw_d(i) = yaw_d(i - 1) + (yaw_rate * uav_dynamics.dt * 2 * pi);
		end
		if yaw_d(i) > pi %bound yaw angle between +-180 degree
			yaw_d(i) = yaw_d(i) - (2 * pi);
		end

		%plan position
		xd(1, i) = radius * cos(circum_rate * uav_dynamics.dt * i * pi);
		xd(2, i) = radius * sin(circum_rate * uav_dynamics.dt * i * pi);
		xd(3, i) = -1;

		%plan velocity
		vd(1, i) = radius * -sin(circum_rate * uav_dynamics.dt * i * pi);
		vd(2, i) = radius * cos(circum_rate * uav_dynamics.dt * i * pi);
		vd(3, i) = 0;
	end

	%plot datas
	time_arr = zeros(1, ITERATION_TIMES);
	accel_arr = zeros(3, ITERATION_TIMES);
	vel_arr.g = zeros(3, ITERATION_TIMES);
	R_arr = zeros(3, 3, ITERATION_TIMES);
	euler_arr = zeros(3, ITERATION_TIMES);
	pos_arr = zeros(3, ITERATION_TIMES);
	W_dot_arr = zeros(3, ITERATION_TIMES);
	W_arr = zeros(3, ITERATION_TIMES);
	M_arr = zeros(3, ITERATION_TIMES);
	prv_angle_arr = zeros(1, ITERATION_TIMES);
	eR_prv_arr = zeros(3, ITERATION_TIMES);
	eR_arr = zeros(3, ITERATION_TIMES);
	eW_arr = zeros(3, ITERATION_TIMES);
	ex_arr = zeros(3, ITERATION_TIMES);
	ev_arr = zeros(3, ITERATION_TIMES);

    m = uav_dynamics.mass;
    g = 9.8;
    Ix = uav_dynamics.J(1, 1);
    Iy = uav_dynamics.J(2, 2);
    Iz = uav_dynamics.J(3, 3);
    
    %construct A matrix
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
         -1/m 0   0   0;
          0   0   0   0;
          0   0   0   0;
          0   0   0   0];
    
    %solve CARE
    [X, L, G] = care(A, B, Q, R);
    K = inv(R) * transpose(B) * X;
    
	for i = 1: ITERATION_TIMES
		%%%%%%%%%%%%%%%%%%%%%%%%%%
		% Update System Dynamics %
		%%%%%%%%%%%%%%%%%%%%%%%%%%
		uav_dynamics = update(uav_dynamics);
        
        %get euler angles from R matrix
        eulers = math.dcm_to_euler(uav_dynamics.R);
        
		%%%%%%%%%%%%%%%%%%%%%%%%%
		% Quadrotor LQR Control %
		%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %system state vector
        x = [eulers(1);
             eulers(2);
             eulers(3);
             uav_dynamics.W(1);
             uav_dynamics.W(2);
             uav_dynamics.W(3);
             uav_dynamics.v(1);
             uav_dynamics.v(2);
             uav_dynamics.v(3);
             uav_dynamics.x(1);
             uav_dynamics.x(2);
             uav_dynamics.x(3)];
         
        %control setpoint vector
        x0 = [deg2rad(0); deg2rad(0); deg2rad(0); 0; 0; 0; 0; 0; 0; 1; 1; -10];

        %control feedforward vector
        u0 = [m*g; 0; 0; 0];
        
        %control vector
        u = -K*[x-x0];
        
        uav_ctrl_f = uav_dynamics.R * [0; 0; u(1)] + [0; 0; -m*g];
        uav_ctrl_M = [u(2); u(3); u(4)];
        
		%feed
		uav_dynamics.M = uav_ctrl_M;
		uav_dynamics.f = uav_ctrl_f;
        
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		% Record datas for plotting %
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		time_arr(i) = i * uav_dynamics.dt;
		time_arr(i) = i * uav_dynamics.dt;
		eR_prv_arr(:, i) = 0; %rad2deg(eR_prv);
		eR_arr(:, i) = 0; %rad2deg(eR);
		eW_arr(:, i) = 0; %rad2deg(eW);
		accel_arr(:, i) = uav_dynamics.a;
		vel_arr.g(:, i) = uav_dynamics.v;
		pos_arr(:, i) = uav_dynamics.x;
		R_arr(:, :, i) = uav_dynamics.R;
		euler_arr(:, i) = rad2deg(math.dcm_to_euler(uav_dynamics.R));
		W_dot_arr(:, i) = rad2deg(uav_dynamics.W_dot);
		W_arr(:, i) = rad2deg(uav_dynamics.W);
		M_arr(:, i) = uav_dynamics.M;
		ex_arr(:, i) = 0; %ex;
		ev_arr(:, i) = 0; %ev;
	end

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Animate the simulation result %
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%rigidbody_visualize([7; 7; 7], pos_arr, R_arr, ITERATION_TIMES, uav_dynamics.dt, 30);

	%%%%%%%%%%%%%%%%%%%%%%%%%%
	%          Plot          %
	%%%%%%%%%%%%%%%%%%%%%%%%%%

	%attitude (euler angles)
		%attitude (euler angles)
	figure('Name', 'attitude (euler angles)');
	subplot (3, 1, 1);
	plot(time_arr, euler_arr(1, :));
	title('attitude (euler angles)');
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

    	%position
	figure('Name', 'position (NED frame)');
	subplot (3, 1, 1);
	plot(time_arr, pos_arr(1, :));
	title('position (NED frame)');
	xlabel('time [s]');
	ylabel('x [m]');
	subplot (3, 1, 2);
	plot(time_arr, pos_arr(2, :));
	xlabel('time [s]');
	ylabel('y [m]');
	subplot (3, 1, 3);
	plot(time_arr, -pos_arr(3, :));
	xlabel('time [s]');
	ylabel('-z [m]');

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
