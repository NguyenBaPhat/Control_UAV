RPM2radps = (2*pi)/60;
radps2RPM = 60/(2*pi);

%% parameter of pelicant
L=0.17;                             % Arm length (m)
Ix= 0.0073; Iy= 0.0073; Iz=0.0117;
I= diag([Ix,Iy,Iz]);                % kg.m^2
m= 0.616;                           % Mass + baterry + Lidar sensor + Atom board
g=9.81;                             % Gravity constant (m/s^2)

%% Aero dynamics
% Convert Rad/s = rpm * 2*pi/160
b=5.9347e-06;                       % N/(rad/s)^2. Thrust coeficients: mg-4b*w0r^2
d= 1.2181e-07;                      % N/(rad/s)^2

%% Initial Condition
omegaInit= [0;0;0];
attitudeInit= [0;0;0];
velInit= [0;0;0];
posInit= [0;0;0];
motorSpeedInit= [0;0;0;];

%% Motor Dynamics
Ks= 735;
tau = 1/200;

%% Controllers
w0 = sqrt(m*g/(4*b)); %rad/s
u0 = 0.49;
lamda = w0^2/u0;
Kth = b*lamda;
Kd = d * lamda;

%% Attitude Control
%for roll
wr_n = 8;
kxir = 0.7;
k1_roll = kxir*wr_n*Ix/(Kth*L)
k2_roll = (wr_n^2)*Ix/(2*k1_roll*Kth*L)
%for pitch
wp_n = 8;
kxip = 0.7;
k1_pitch = kxip*wp_n*Iy/(Kth*L)
k2_pitch = (wp_n^2)*Iy/(2*k1_pitch*Kth*L)
%for yaw
wy_n = 8;
kxiy = 0.7;
k1_yaw = kxiy*wy_n*Iz/(2*Kd)
k2_yaw = (wy_n^2)*Iz/(4*k1_yaw*Kd)
%altitude controller
wh_n=10;
kxih=0.75;
kp_h=2*wh_n*kxih*m/(4*Kth)
ki_h=0.04;
%for x controller
wposition_nx=1.5;
kxi_positionx=0.9;
kp_x=2*g/wposition_nx^2
kd_x=4*g/(kxi_positionx*wposition_nx)
%for y controller
wposition_ny=1;
kxi_positiony=0.5;
kp_y=2*g/wposition_ny^2
kd_y=4*g/(kxi_positiony*wposition_ny)

