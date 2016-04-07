Kinematics library

This folder contains kinematics library

Haoran Yu

3/31/2016

1. Arm_Kinematics.m: define the class Arm_Kinematics that contains all the information for URDF.

2. Arm_Kinematics.calc_FK: returns 4 by 4 by n homogeneous transformation matrices.

3. calc_Jacobian_all.m: this code takes Frames and returns analytical Jacobian.

	---J_car: Cartesian Jacobian(at spherical roll joint, 6 by 5)
	
	---J_rcm: RCM Jacobian (at distal wrist joint, 6 by 6)

	---J_all: overall Jacobian (6 by 11).

4. calc_Jacobian_6DoF_rcm:

	---J_car_6DoF_rcm: 6DoF Caresian Jacobian (6 by 6) with rcm position at rcm link and orientation parallel to spherical roll joint.

5. calc_Jacobian_general.m:

	---J_touch: touch point Jacobian with given touch point position and selected frames.

6. convert2rcm.m: takes 11 DoF joint input and convert to 13 DoF joint input

7. compute_twist: given current and target position and orientation, calculate the eef twist and position and orientation error.

10. CPM.m: cross product matrix.

