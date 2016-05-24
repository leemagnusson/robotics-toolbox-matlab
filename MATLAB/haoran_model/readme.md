This folder contains kinematics foundation and simulation codes.

How to get the code from gerrit:

1. get access to verb surgical gerrit code base.

2. Go to https://gerrit.verbsurgicaleng.com/g/plugins/gitiles/docs/+/master/README.md

3. Setup the gerrit following Developer Setup

4. clone rouxeny/arm_analysis with change-ID

git clone ssh://Haoran8879@gerrit.verbsurgicaleng.com:29418/rouxeny/arm_analysis && scp -p -P 29418 Haoran8879@gerrit.verbsurgicaleng.com:hooks/commit-msg arm_analysis/.git/hooks/

6/1/2016

1. StartUp.m: this is a script that initiate the folder. The code use "URDF.m" from matlab robotics toolbox and convert URDF file into matlab urdf data format. This script also generate point clouds and boundary boxes for collision detection.

   Input:

   0 --- Do not change model and only add the path

   1 --- Load Ver 1.0 Arm

   1.5 --- Load Ver 1.5. Arm

2. Test files

	a. TestDrawOneArm.m: draw single robot at different joint configuration.

	b. TestInverseKinematics.m: run resolved rates by giving the end effector a target position and orientation.
    
    c. TestDrawThreeArm.m: Draw three arms with hernia setup

3. Init folder: init files for procedure setup, IK and bed/bed adapter.
	a. InitProcedureSetup.m: get all the SW input and setup different procedure trocar, arm base, init guess of joint angles and target position

	b. InitIKParameters.m: init IK dt, velocity, convergence radius and etc

    d. InitBed.m: init bed and bed adapter

4. projects folder: Analysis files:

	a. AnalysisHerniaCollisionDetection.m: analyze the collision detection with hernia setup and hernia log.

	b. AnalysisHerniaProcedureSetup.m: Simulate the automatic hernia setup with estimate from joint angles.

	c. AnalysisWorkspaceEEF.m: analyze and plot the EEF workspace with fixed RCM

	d. AnalysisWorkspaceRCM.m: analyze and plot the Trocar workspace

5. procedure_ui folder: Arm_setup_UI: GUI for arm setup

6. lib: library files

7. data: saved mat files to run projects and ui

8. figures: procedure setup figures

9. 2015_drawer_urdf_3a: table and table adapter urdf and meshes

10. Arm_version_1.0: Ver 1.0 Arm urdf and meshes

11. Arm_version_1.5: Ver 1.5 Arm urdf and meshes

12. workspace folder: meshes for macro workspace analysis

13. export folder: scripts for loading the logs.

	a. InitLogToolPath.m: init tool path from log files
