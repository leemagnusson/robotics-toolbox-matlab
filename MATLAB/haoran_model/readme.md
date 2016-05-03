This folder contains kinematics foundation and simulation codes

4/7/2016

1. StartUp.m: this is a script that initiate the folder. The code use "URDF.m" from matlab robotics toolbox and convert URDF file into matlab urdf data format. This script also generate point clouds and boundary boxes for collision detection.

    --- Arm_version_1.0 is the URDF and robot folder

    --- 2015_drawer_urdf_3a: bed and bed adapter urdf and robot folder

    --- change do_save to 0 since all the URDF have already been created.

2. Test files

	a. TestDrawOneArm.m: draw single robot at different joint configuration.

	b. TestInverseKinematics.m: run resolved rates by giving the end effector a target position and orientation.
    
    c. TestDrawThreeArm.m: Draw three arms with hernia setup

3. Init files:
	a. InitHerniaSetup.m: get all the SW input and setup hernia trocar, arm base, init guess of joint angles and hernia position

	b. InitIKParameters.m: init IK dt, velocity, convergence radius and etc

	c. InitLogToolPath.m: init tool path from log files

    d. InitBed.m: init bed and bed adapter

4. Analysis files:

	a. AnalysisHerniaCollisionDetection.m: analyze the collision detection with hernia setup and hernia log.

	b. AnalysisHerniaProcedureSetup.m: Simulate the automatic hernia setup with estimate from joint angles.

	c. AnalysisWorkspaceEEF.m: analyze and plot the EEF workspace with fixed RCM

	d. AnalysisWorkspaceRCM.m: analyze and plot the Trocar workspace

5. Arm_setup_UI: GUI for arm setup

    --- The user guide is in report/GraphicUserInterface

6. lib: library files

7. data: saved mat files

8. report: documentation folder
