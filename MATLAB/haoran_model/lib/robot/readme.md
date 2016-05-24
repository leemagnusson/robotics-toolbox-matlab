Kinematics library

This folder contains kinematics library

Haoran Yu

3/31/2016

1. RobotKinematics.m: define the class RobotKinematics that contains all the information for URDF.

2. RobotKinematics.CalculateFK: returns 4 by 4 by n homogeneous transformation matrices.

3. CalculatJacobianAll.m: this code takes Frames and returns analytical Jacobian.

	---jacobian_cartesian: Cartesian Jacobian(at spherical roll joint, 6 by 5)
	
	---jacocbian_rcm: RCM Jacobian (at distal wrist joint, 6 by 6)

	---jacobian_all: overall Jacobian (6 by 11).

4. CalculatJacobian6DofRDM:

	---jacobian_6dof_rcm: 6DoF Caresian Jacobian (6 by 6) with rcm position at rcm link and orientation parallel to spherical roll joint.

5. CalculateJacobianGeneral.m:

	---jacobian_touch: touch point Jacobian with given touch point position and selected frames.

6. ConvertToRcm.m: takes 11 DoF joint input and convert to 13 DoF joint input

7. ComputeTwist: given current and target position and orientation, calculate the eef twist and position and orientation error.

