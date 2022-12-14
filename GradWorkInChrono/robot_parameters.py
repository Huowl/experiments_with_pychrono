import numpy as np

ground = [20, 1.5, 0.05] # [m]
z_offset = 0.35 # [m]

# World Parameters
world_damphing = 20
world_rot_damphing = 10

# Model Unitree A1

body_dim = [0.267, 0.194, 0.114]
body_mass = 6.0
body_density = body_mass / np.prod(body_dim)

femur_length = 0.2 # [m]
femur_width = 0.0245 # [m]
femur_height = 0.034 # [m]
femur_mass = 0.888; # [kg]
femur_density = (femur_mass
                   / femur_length / femur_width / femur_height)

#femParam = [femur_length, femur_width, femur_height, femur_mass];

tibia_length = 0.2 # [m]
tibia_width = 0.016
tibia_height = 0.016
tibia_mass = 0.151; # [kg]

tibia_density = (tibia_mass
                   / tibia_length / tibia_width / tibia_height)

#tibParam = [tibia_length, tibia_width, tibia_height, tibia_mass];

hip_radius = 0.046 # [m]
hip_length = 0.04
hip_mass = 0.595
hip_offset = 0.065

hip_density = hip_mass / (np.pi*hip_radius**2*hip_length)

leg_offset_x = 0.1805
leg_offset_y = 0.047

foot_mass = 0.06
foot_radius = 0.01

foot_density = foot_mass / (4/3*np.pi*foot_radius**3)

#offset_RF = [leg_offset_x, -leg_offset_y, body_dim[1]/2]
#offset_LF = [leg_offset_x, leg_offset_y, body_dim[1]/2]

#offset_RR = [-leg_offset_x, -leg_offset_y, body_dim[1]/2]
#offset_LR = [-leg_offset_x, leg_offset_y, body_dim[1]/2]

'''
%% Motor Parameters Unitree A1 20 march 2022

PWM_freq = 100; % [Hz]
max_torque = 33.5; % [V] Scaling PWM 100
m_voltage = 24; % [V] Output PWM

arm_resistance = 4.1730; %[Ohm]
arm_inductance = 250e-6; % [H]
torque_c = 0.8372; %[N*m/A]

rotor_inertia = 72e-6;
rotor_damping = 0.09;

g_ratio = 3;%10.3;

stall_torque_motor = 33.5; % [H*m]
no_load_speed_motor = 21; % [rad/s]

%% Spine parameters

% bd_spine_eq_pos = 0; %[rad] equlibrium position spine
% bd_spine_stiffness = 300; % [Hm/rad] spine joing spring stifness
% bd_spine_damping = 10; %  [Hm/(rad/s)] spine joint damphing coefficient
% Bounds [rad]
spine_pos_low_limit = -pi/2;
spine_pos_up_limit = pi/2;
'''

# Contact
sp_rad = 0.02
pl_x = ground[0]
pl_y = ground[1]
pdepth = 0.03

cntstiff = (body_mass/4 + femur_mass + tibia_mass + hip_mass)*10/0.001 # 1e5 past 03.04
cntdmp = cntstiff/10 #1e5

staticFriction = 0.9
dynamicFriction = 0.8
mu_vth = 1e-1

max_height_foot = 0.27