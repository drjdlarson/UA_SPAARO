%Initialize a vehicle for use in the simulation
%
%
% Joshua Briggs
% Reference paper: "Design, modeling and control of an omni-directional aerial vehicle"


%% Platform's name
Aircraft.name = 'seldom';

%% Expectation for parameters under the telemetry bus
% Param 01 is the angle from the NED frame to the local nav frame. This
% will make more sense when reading the Inceptor Configuration section below

%% Propulsion properties
% Number of motors
Aircraft.Motor.nMotor = 8;

% Assign a pwm channel to motor
Aircraft.Motor.map = [ 1 ; 2 ; 3 ; 4 ; 5 ; 6 ; 7 ; 8];

% Motor positions relative to c.g in [m] [x,y,z](section 2 of reference paper)
% Per Figure 3 in the paper, x is +fwd, y is +left, z is +up
% We  need to convert these to a tradition body axis where x is +fwd, y is
% +right, z is +down, so need to flip y and z axes.
Aircraft.Motor.pos_m = 1/sqrt(3)*[ 1  1  1;...
                                  -1  1  1;...
                                   1 -1  1;...
                                  -1 -1  1;...
                                   1  1 -1;...
                                  -1  1 -1;...
                                   1 -1 -1;...
                                  -1 -1 -1];
Aircraft.Motor.pos_m(:,2:3) = -1*Aircraft.Motor.pos_m(:,2:3);
% Motor alignment per section 2 of the reference paper.
% Per Figure 3 in the paper, x is +fwd, y is +left, z is +up
% We  need to convert these to a tradition body axis where x is +fwd, y is
% +right, z is +down, so need to flip y and z axes.
a = 1/2 + 1/sqrt(12);
b = 1/2 - 1/sqrt(12);
c = 1/sqrt(3);
Aircraft.Motor.align = [-a  b  c;...
                         b  a -c;...
                        -b -a -c;...
                         a -b  c;...
                         a -b  c;...
                        -b -a -c;...
                         b  a -c;...
                        -a  b  c];
Aircraft.Motor.align(:,2:3) = -1*Aircraft.Motor.align(:,2:3);
clear a b c

% Motor rotation direction (positive clockwise)
Aircraft.Motor.dir = ones(8,1);

% Motor time constant assuming a first order response. Estimated from step
% responses in section 2 of the reference paper.
Aircraft.Motor.Tau = 0.15;
% Min and max thrust command allowed per motor from section 5A of the
% reference paper.
Aircraft.Motor.fmin_N = 0.2; % Newtons
Aircraft.Motor.fmaxa_N = 6.7; % Newons
% We shold test our actual motors+props and come up with a PWM vs voltage
% or ND to voltage curve plus step response characteristics.

%% Mass properties (estimated from Section 5.B of reference paper) CG is at body origin
% Mass [kg]
Aircraft.Mass.mass_kg = 0.886;
% c.g. location [m]
Aircraft.Mass.cg_m = [0 0 0];
Aircraft.Mass.MotorPlusProp = 0.0455 + 0.0105;
% Moments of inertia [kg*m^2]
% Best estimate is to treat motor and propeller weights as point masses.
% With knowing nothing about the exact size/density of the other components
% but knowing they are generally as close as possible to the CG, I'm making
% a complete guess that they add ~20% to the inertia.

inertia = [0 0 0];
for idx_motor = 1:Aircraft.Motor.nMotor
    inertia = inertia + Aircraft.Mass.MotorPlusProp*Aircraft.Motor.pos_m(idx_motor,:).^2;
end

Aircraft.Mass.ixx_kgm2 = inertia(1);
Aircraft.Mass.iyy_kgm2 = inertia(2);
Aircraft.Mass.izz_kgm2 = inertia(3);
Aircraft.Mass.inertia_kgm2 = [Aircraft.Mass.ixx_kgm2           0                         0;...
                              0                       Aircraft.Mass.iyy_kgm2             0;...
                              0                                0                Aircraft.Mass.izz_kgm2];
Aircraft.Mass.inertia_inverse = inv(Aircraft.Mass.inertia_kgm2);
clear inertia

%% Geometric properties of the body
% Axial area (m^2) in body frame
% Frontal area at different angles
% Aircraft.Geo.front_area_m2 = [1];
    
%% Aerodymanics coef
% Axis system for aerodynamic coefficients
% https://www.mathworks.com/help/aeroblks/aerodynamicforcesandmoments.html
% 1 = Wind axis
% 2 = Stability axis
% 3 = Body axis
Aircraft.Aero.axis = 1;
%Drag coefficient
% Aircraft.Aero.Cd = 0.8; % Based on CD of slanted cube [Jan Willem Vervoorst]

%% Inceptor Configuration
% Configure function of main control
% Other channels are available as raw 172-1811 for FrSky sbus

% Since the first step is to fly in manual mode and it will be hard to tell
% which way is "forward" with an omnicopter, thinking about 2 control modes
% for manual flight. One where you control velocity (translational and
% rotational) about the body axis, and the second where the velocities are
% about a local navigation axis that is NED with the heading rotated to a
% parameter defined angle. This allows one to configure it to align with
% the walls in a room for example.

Aircraft.Inceptor.vel_U_cmd_fps = 1;
Aircraft.Inceptor.roll_rate = 2; % left roll/pitch/yaw same channels as super
Aircraft.Inceptor.pitch_rate = 3;
Aircraft.Inceptor.yaw_rate = 4;
Aircraft.Inceptor.mode = 5;
% Mode = 0: fly in body axis
% Mode = 1: fly in local nav axis
% Aircraft.Inceptor.relay = 6;
Aircraft.Inceptor.throttle_e_stop = 7;
Aircraft.Inceptor.vel_N_cmd_fps = 8;
Aircraft.Inceptor.vel_E_cmd_fps = 9;

% Inceptor deadband
Aircraft.Inceptor.deadband = 0.05;

%% Effectors
% Number of PWM channels
Aircraft.Eff.nPwm = 8;
% Number of SBUS channels
Aircraft.Eff.nSbus = 16; % Not sure what this is. Left it alone from Super.
% Total number of channels
Aircraft.Eff.nCh = Aircraft.Eff.nPwm + Aircraft.Eff.nSbus;
% PWM Range
Aircraft.PWM.min = 1100;
Aircraft.PWM.max = 1900;
Aircraft.PWM.norm_min = -1;
Aircraft.PWM.norm_max = 1;

%% Propeller 
% %Diameter [inches]
% Aircraft.Prop.dia_in = 32;
% 
% % Coefficient of thrust constant obtained from T-motor's website data
% Aircraft.Prop.kt = 0.0388;   %N-m/N
% 
% %Polynomial coefficient for simple thrust model
% Aircraft.Prop.poly_thrust = [109.86 -18.329];
% Aircraft.Prop.poly_torque = [4.2625 -0.7112];

%% Battery
% % Number of battery cells
% Aircraft.Battery.nCell = 12;
% % Maximum voltage per cell [V]
% Aircraft.Battery.volt_per_cell = 4.2;
% % Voltage available
% Aircraft.Battery.voltage = Aircraft.Battery.nCell * Aircraft.Battery.volt_per_cell;
Aircraft.Battery.Voltage = 5; % Completely made up a number to make the sim happy. Not using it for now so not sure if it's needed eventually or not.
% % Power module voltage gain. Gain between battery voltage and voltage
% % output by power modele
% Aircraft.Battery.voltage_gain = 18.95;
% % Power module current to voltage gain. Gain between current draw and
% % voltage output by power module
% Aircraft.Battery.current_to_voltage_gain_vpma = 125.65 * 1000; %mA per volt

%% Sensors (not touched from Super, probably not right, only used on the sim side, probably good enough)
% MPU-9250 IMU
% Accel
Aircraft.Sensors.Imu.Accel.scale_factor = eye(3);
Aircraft.Sensors.Imu.Accel.bias_mps2 = [0 0 0]';
Aircraft.Sensors.Imu.Accel.noise_mps2 = 0.0785 * ones(3, 1);
Aircraft.Sensors.Imu.Accel.upper_limit_mps2 = 156.9064 * ones(3, 1);
Aircraft.Sensors.Imu.Accel.lower_limit_mps2 = -1 * Aircraft.Sensors.Imu.Accel.upper_limit_mps2;
% Gyro
Aircraft.Sensors.Imu.Gyro.scale_factor = eye(3);
Aircraft.Sensors.Imu.Gyro.bias_radps = [0 0 0]';
% G-sensitivity in rad/s per m/s/s
Aircraft.Sensors.Imu.Gyro.accel_sens_radps = [0 0 0]';  
Aircraft.Sensors.Imu.Gyro.noise_radps = deg2rad(0.1) * ones(3, 1);
Aircraft.Sensors.Imu.Gyro.upper_limit_radps = deg2rad(2000) * ones(3, 1);
Aircraft.Sensors.Imu.Gyro.lower_limit_radps = -1 * Aircraft.Sensors.Imu.Gyro.upper_limit_radps;
% Magnetometer
Aircraft.Sensors.Imu.Mag.scale_factor = eye(3);
Aircraft.Sensors.Imu.Mag.bias_ut = [0 0 0]';
Aircraft.Sensors.Imu.Mag.noise_ut =  0.6 * ones(3, 1);
Aircraft.Sensors.Imu.Mag.upper_limit_ut =  4800 * ones(3, 1);
Aircraft.Sensors.Imu.Mag.lower_limit_ut = -1 * Aircraft.Sensors.Imu.Mag.upper_limit_ut;
% GNSS model
Aircraft.Sensors.Gnss.sample_rate_hz = 5;
Aircraft.Sensors.Gnss.fix = 3; % 3D fix
Aircraft.Sensors.Gnss.num_satellites = 16;
Aircraft.Sensors.Gnss.horz_accuracy_m = 1.5;
Aircraft.Sensors.Gnss.vert_accuracy_m = 5.5;
Aircraft.Sensors.Gnss.vel_accuracy_mps = 0.05;
Aircraft.Sensors.Gnss.track_accuracy_rad = deg2rad(2);
Aircraft.Sensors.Gnss.hdop = 0.7;
Aircraft.Sensors.Gnss.vdop = 0.7;
% Air data model
% Static pressure
Aircraft.Sensors.PitotStaticInstalled = 0;
Aircraft.Sensors.StaticPres.scale_factor = 1;
Aircraft.Sensors.StaticPres.bias_pa = 0;
Aircraft.Sensors.StaticPres.upper_limit_pa = 120000;
Aircraft.Sensors.StaticPres.lower_limit_pa = 70000;
% 1% of the full-scale range
Aircraft.Sensors.StaticPres.noise_pa = 0.01 * (Aircraft.Sensors.StaticPres.upper_limit_pa - Aircraft.Sensors.StaticPres.lower_limit_pa);
% Differential pressure
Aircraft.Sensors.DiffPres.scale_factor = 1;
Aircraft.Sensors.DiffPres.bias_pa = 0;
Aircraft.Sensors.DiffPres.upper_limit_pa = 1000;
Aircraft.Sensors.DiffPres.lower_limit_pa = 0;
% 2% of the full-scale range
Aircraft.Sensors.DiffPres.noise_pa =  0.02 * (Aircraft.Sensors.DiffPres.upper_limit_pa - Aircraft.Sensors.DiffPres.lower_limit_pa);

%% Controller parameters
% allow values to come from telem bus or be hardcoded from this file
Aircraft.Control.hardcode_values = false;

% Motor minimum throttle 
%%%% NOTE %%%% It's unclear to me if the motors/props on the vehicle can
%%%% spin both directions. If so, set spin_min to -0.95 If not, set to 0 or
%%%% 0.05 if you don't want it to fully stop spinning to prevent any chance
%%%% of lock-up
Aircraft.Control.motor_spin_min = -0.95;

% Motor maximum throttle
% Not sure why super had motors limited away from 100%. Leaving it in in case it was for heat or motor lifespan.
Aircraft.Control.motor_spin_max = 0.95;

% Thorttle stick minimum
% Mimimum valid value of the throttle stick so that arming and gain reset
% occurs
Aircraft.Control.throttle_min = 0.05;

% Motor ramp time [s]
% Time so slowly ramp motor from 0 to motor_spin_min. Prevent initial
% voltage spike
Aircraft.Control.motor_ramp_time_s = 3;

%% Mixing
% B matrix (effectivity matrix) buildup. How much body F and M each
% motor/prop generates.
% Mixer works in normalized PWM, so the denominator would just be 1.
% This is the amount of thrust in Newtons you get per unit of normalized
% PWM.
Aircraft.Mixing.ThrustPerNormPWM = Aircraft.Motor.fmaxa_N;
% Multiply by each motor's normal vector to get force in body coordinates
Aircraft.Mixing.B(1:8,1:3) = Aircraft.Mixing.ThrustPerNormPWM*Aircraft.Motor.align;
% Divide by mass for acceleration
Aircraft.Mixing.B(1:8,1:3) = Aircraft.Mixing.B(1:8,1:3)/Aircraft.Mass.mass_kg;
% Using the moment arm (which is just the position) crossed with the normal
% unit vector to get the unit vector for the moment distribution.
Aircraft.Mixing.B(1:8,4:6) = Aircraft.Mixing.ThrustPerNormPWM*cross(Aircraft.Motor.pos_m,Aircraft.Motor.align);
% "Divide" by inertia for angular acceleration
for idx = 1:Aircraft.Motor.nMotor
    Aircraft.Mixing.B(idx,4:6) = (Aircraft.Mass.inertia_inverse*Aircraft.Mixing.B(idx,4:6)')';
end

%% Command Limits
Aircraft.Control.Limits.velocity_mps = 3; % meters per second
Aircraft.Control.Limits.angular_rate_rps = pi; % full rotation in 2 seconds

%% Sim Parameters
Target.alt_msl_m = 0;
Target.lat_deg = 33.21135;
Target.lon_deg = -87.54013;
Env.wmm_nt = wrldmagm(Target.alt_msl_m + geoidheight(Target.lat_deg, constrain360(Target.lon_deg), 'EGM2008'), Target.lat_deg, Target.lon_deg, decyear(now));

%%
% %% Yaw rate controller parameters
% % Max yaw rate [radps]
% Aircraft.Control.yaw_rate_max = 1.74533; %~100deg/s
% % It's good to limit the maximum yaw rate because excessive yaw rate may
% % cause some motors to slow down too much that hover cannot be maintained
% 
% % Yaw accel PI gains
% Aircraft.Control.P_yaw_rate = 0.5;
% Aircraft.Control.I_yaw_rate = 0.05;
% Aircraft.Control.D_yaw_rate = 0.02;
% 
% %% Pitch controller parameters
% % Max pitch angle [rad]
% Aircraft.Control.pitch_angle_lim = 0.35;  %~20deg
% 
% % Pitch cmd controller gains
% Aircraft.Control.P_pitch_angle = 0.04;
% Aircraft.Control.I_pitch_angle = 0.04;
% Aircraft.Control.D_pitch_angle = 0.02;
% 
% % Max pitch rate [radps]
% Aircraft.Control.pitch_rate_max = 1; %~60deg/s
% 
% %% Roll controller parameters
% % Max roll angle [rad]
% Aircraft.Control.roll_angle_lim = 0.35;  %~20deg
% 
% % Roll cmd controller gains
% Aircraft.Control.P_roll_angle = 0.04;
% Aircraft.Control.I_roll_angle = 0.04;
% Aircraft.Control.D_roll_angle = 0.02;
% 
% % Max roll rate [radps]
% Aircraft.Control.roll_rate_max = 1; %~60deg/s
% 
% %% Vertical speed controller parameters
% Aircraft.Control.est_hover_thr = 0.6724;
% % Vertical speed limit [m/s]
% Aircraft.Control.v_z_up_max = 2;
% Aircraft.Control.v_z_down_max = 1; %minimum of -1 m/s
% % Vertical speed controller gain
% Aircraft.Control.P_v_z = 0.09;
% Aircraft.Control.I_v_z = 0.05;
% Aircraft.Control.D_v_z = 0.005;
% 
% %% Translational speed controller parameters
% % Horizontal spped limit [m/s]
% Aircraft.Control.v_hor_max = 5;
% 
% % Horizontal speed controller gain
% Aircraft.Control.P_v_hor = 0.5;
% Aircraft.Control.I_v_hor = 0.01;
% Aircraft.Control.D_v_hor = 0.1;
% 
% 
% %% Altitude controller parameters
% Aircraft.Control.P_alt = 1;
% Aircraft.Control.I_alt = 0.2;
% 
% %% Distance controller parameters
% Aircraft.Control.P_xy = 3;
% Aircraft.Control.I_xy = 0.1;
% Aircraft.Control.wp_radius = 1.5;
% Aircraft.Control.wp_nav_speed = 3;
% 
% %% Heading controller parameters
% Aircraft.Control.P_heading = 1;
% Aircraft.Control.I_heading = 0.01;
% Aircraft.Control.D_heading = 0.01;
% 
% %% Return to land parameters
% Aircraft.Control.rtl_altitude = 35;
% Aircraft.Control.land_speed_fast = 1;
% Aircraft.Control.land_speed_slow = 0.3;
% Aircraft.Control.land_slow_alt = 10; % Altitude [m] where precision landing is engaged