Data store folder

This folder stores data

Haoran Yu

4/29/2016

Important ones

1. arm_version_1.0.mat: robot_kinematics is the class that contains URDF information

2. coupling_matrix.mat: 13 by 11 coupling matrix

3. parent_number.mat: parent numbers and joint numbers for different URDF

4. point_boundary_all.mat: all arms' bounding box

5. q_init_setup_....mat: init joint values for different setup

6. hernia_ool_path.mat: tool path from log that does suture.

7. vertex_hernia_patient_body.mat: hernia body vertex data at origin

8. vertex_arm_origin.mat: Arm link vertex data at origin

9. urdf_info.mat: URDF joint, link information parsed from urdf file.

10. vertex_bed.mat: bed model and bed base coordinate

11. vertex_bed_adapter.mat: bed adapter model and bed adapter base frame

12. bed_adapter_info.mat: bed adapter rpy and xyz for FK.