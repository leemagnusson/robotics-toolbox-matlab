function q = ArmRepositioning(q,separation_angle,selected_axis)
load('arm_version_1.0.mat')
robot_object.transformation_base_=eye(4);
dt = 0.001;
steps = 200;
angular_velocity = separation_angle/(steps*dt);
for j=1:steps
    robot_object.CalculateFK(q);
    [jacobian_rcm,jacobian_cartesian,jacobian_all] = robot_object.CalculateJacobianAll;
    jacobian_car_6DoF = robot_object.CalculateJacobian6DofRCM;
    jacobian_temp = pinv(robot_object.frames_(1:3,1:3,7)) * jacobian_all(4:6,1:6);
    jacobian_rcm_rotation = jacobian_temp(1:2,:);
    if selected_axis == 'x'
        twist_repositioning = [zeros(9,1);angular_velocity;0];
    else
        twist_repositioning = [zeros(9,1);0;angular_velocity];
    end
    jacobian_repositioning = [jacobian_all;jacobian_car_6DoF(1:3,:) zeros(3,5);jacobian_rcm_rotation zeros(2,5)];
    q_dot = pinv(jacobian_repositioning) * twist_repositioning;
    q = q + q_dot*dt;
end
q = WrapToPi(q);