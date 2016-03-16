haoran_model

This folder contains kinematics simulation codes written by Haoran Yu

by 3/16/2016 the codes have not been commented

3/16/2016

1. start_up_haoran.m: this is a script

2. test.m: import URDF and draw robot at home configuration.

3. resolved_rates.m: run resolved rates by giving the end effector a commanded velocity.

4. resolved_rates_col_detection.m: run resolved rates on two arms and detect contact by showing "colision" as true or false in each arm configuration.

5. The code use "URDF.m" from matlab robotics toolbox and convert URDF file into matlab urdf data format

6. Arm_Kinematics.m: define the class Arm_Kinematics that contains information of forward kinematics that is transformed from URDF information and basic arm dynamics parameter.

7. stl2matlab.m transformSTL.m and plotSTL.m: these are functions from igesToolbox that load, transform and plot stl files. The toolbox could be downloaded from http://www.mathworks.com/matlabcentral/fileexchange/13253-iges-toolbox

8. calc_Jacobian_all.m: this code takes Arm_Kinematics and returns analytical Jacobian.

---J_car: Cartesian Jacobian(at spherical roll joint, 6 by 5)

---J_rcm: RCM Jacobian (at distal wrist joint, 6 by 6)

---J_car_6DoF: 6DoF Caresian Jacobian(at spherical roll joint, 6 by 6)

---J_all: overall Jacobian (6 by 11).

9. convert2rcm.m: takes 11 DoF joint input and convert to 13 DoF joint input

10. CPM.m: cross product matrix.

