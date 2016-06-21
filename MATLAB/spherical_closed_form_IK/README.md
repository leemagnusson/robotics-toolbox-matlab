Code for calculating and testing the closed form inverse kinematics of the spherical arm.
This document describes the steps:
https://drive.google.com/drive/folders/0B_fDry3ewv7rNnBKWlpMcWN5dGM

Run TestIK to generate random poses and test forward and inverse kinematics. It generates random poses within bounds of the joints and tests forward and inverse kinematics.

The scripts heavily use symbolic expressions that were calculated offline to avoid online calculations for every pose. For this reason the matrices consist of lengthy symbolic expressions. 

TODO:
Detect and handle all singularities
