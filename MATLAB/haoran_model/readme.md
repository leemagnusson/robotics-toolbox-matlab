haoran_model

This folder contains kinematics foundation and simulation codes written by Haoran Yu

3/31/2016

1. startup_haoran.m: this is a script that initiate the folder. The code use "URDF.m" from matlab robotics toolbox and convert URDF file into matlab urdf data format. This script also generate point clouds and boundary boxes for collision detection.

2. Test files

	a. Test_Draw_arm.m: draw single robot at different joint configuration.

	b. Test_Resolved_rates.m: run resolved rates by giving the end effector a target position and orientation.

3. Init files:
	a. init_Hernia_setup.m: get all the SW input and setup hernia trocar, arm base, init guess of joint angles and hernia position

	b. init_IK_parameters.m: init IK dt, velocity, convergence radius and etc

	c. init_Tool_path.m: init tool path from log files

4. Analysis files:
	a. Analysis_Collision_detection_hernia.m: analyze the collision detection with hernia setup and hernia log.

	b. Analysis_Hernia_setup.m: Simulate the automatic hernia setup with estimate from joint angles.

	c. Analysis_EEF_workspace.m: analyze and plot the EEF workspace with fixed RCM

	d. Analysis_RCM_workspace.m: analyze and plot the Trocar workspace

	e. Analysis_Reach_study_hernia.m: analyze the reach study with hernia setup. Being developed. 

5. 

6. Arm_Kinematics.m: define the class Arm_Kinematics that contains information of forward kinematics that is transformed from URDF information and basic arm dynamics parameter.

7. stl2matlab.m transformSTL.m and plotSTL.m: these are functions from igesToolbox that load, transform and plot stl files. The toolbox could be downloaded from http://www.mathworks.com/matlabcentral/fileexchange/13253-iges-toolbox

8. calc_Jacobian_all.m: this code takes Arm_Kinematics and returns analytical Jacobian.

---J_car: Cartesian Jacobian(at spherical roll joint, 6 by 5)

---J_rcm: RCM Jacobian (at distal wrist joint, 6 by 6)

---J_car_6DoF: 6DoF Caresian Jacobian(at spherical roll joint, 6 by 6)

---J_all: overall Jacobian (6 by 11).

9. convert2rcm.m: takes 11 DoF joint input and convert to 13 DoF joint input

10. CPM.m: cross product matrix.

