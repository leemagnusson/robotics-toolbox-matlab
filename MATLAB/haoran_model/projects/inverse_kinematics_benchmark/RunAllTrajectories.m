function RunAllTrajectories()

% run predefine trajectory first
%%
p0 = zeros(11,1);
p0(8) = -0.07;
p0(10:11) = pi/2;
InverseKinematicBenchmark( p0, 14, 0.5, 'Projected Gauss Seidel SOR with Limits', 'rotate_x','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Projected Gauss Seidel SOR with Limits', 'rotate_y','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Projected Gauss Seidel SOR with Limits', 'rotate_z','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Projected Gauss Seidel SOR with Limits', 'x_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Projected Gauss Seidel SOR with Limits', 'y_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Projected Gauss Seidel SOR with Limits', 'z_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Projected Gauss Seidel SOR with Limits', 'rotate_x_and_x_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Projected Gauss Seidel SOR with Limits', 'rotate_y_and_y_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Projected Gauss Seidel SOR with Limits', 'rotate_z_and_z_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Projected Gauss Seidel SOR with Limits', 'move_to_rcm','absolute', 2);
%%
InverseKinematicBenchmark( p0, 14, 0.5, 'Projected Gauss Seidel SOR with Limits', 'rotate_x','relative', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Projected Gauss Seidel SOR with Limits', 'rotate_y','relative', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Projected Gauss Seidel SOR with Limits', 'rotate_z','relative', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Projected Gauss Seidel SOR with Limits', 'x_motion','relative', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Projected Gauss Seidel SOR with Limits', 'y_motion','relative', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Projected Gauss Seidel SOR with Limits', 'z_motion','relative', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Projected Gauss Seidel SOR with Limits', 'rotate_x_and_x_motion','relative', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Projected Gauss Seidel SOR with Limits', 'rotate_y_and_y_motion','relative', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Projected Gauss Seidel SOR with Limits', 'rotate_z_and_z_motion','relative', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Projected Gauss Seidel SOR with Limits', 'move_to_rcm','relative', 2);
%%
InverseKinematicBenchmark( p0, 14, 0.5, 'Gauss Seidel SOR', 'rotate_x','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Gauss Seidel SOR', 'rotate_y','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Gauss Seidel SOR', 'rotate_z','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Gauss Seidel SOR', 'x_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Gauss Seidel SOR', 'y_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Gauss Seidel SOR', 'z_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Gauss Seidel SOR', 'rotate_x_and_x_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Gauss Seidel SOR', 'rotate_y_and_y_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Gauss Seidel SOR', 'rotate_z_and_z_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Gauss Seidel SOR', 'move_to_rcm','absolute', 2);
%%
InverseKinematicBenchmark( p0, 14, 0.5, 'SVD', 'rotate_x','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'SVD', 'rotate_y','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'SVD', 'rotate_z','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'SVD', 'x_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'SVD', 'y_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'SVD', 'z_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'SVD', 'rotate_x_and_x_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'SVD', 'rotate_y_and_y_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'SVD', 'rotate_z_and_z_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'SVD', 'move_to_rcm','absolute', 2);

%%
InverseKinematicBenchmark( p0, 14, 0.5, 'Damped Least Square', 'rotate_x','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Damped Least Square', 'rotate_y','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Damped Least Square', 'rotate_z','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Damped Least Square', 'x_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Damped Least Square', 'y_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Damped Least Square', 'z_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Damped Least Square', 'rotate_x_and_x_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Damped Least Square', 'rotate_y_and_y_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Damped Least Square', 'rotate_z_and_z_motion','absolute', 2);
InverseKinematicBenchmark( p0, 14, 0.5, 'Damped Least Square', 'move_to_rcm','absolute', 2);

% now run the suturing log
%%
p0(10) = pi/3;
p0(11) = pi/3;
file_name = '160721_anette_egg_sponge_3-1.txt';
InverseKinematicBenchmarkPlayback( p0, 14, 'Projected Gauss Seidel SOR with Limits', file_name, 'absolute', 2);
InverseKinematicBenchmarkPlayback( p0, 14, 'Gauss Seidel SOR', file_name, 'absolute', 2);
InverseKinematicBenchmarkPlayback( p0, 14, 'SVD', file_name, 'absolute', 2);
InverseKinematicBenchmarkPlayback( p0, 14, 'Damped Least Square', file_name, 'absolute', 2);
%%
p0(10) = pi/3;
p0(11) = pi/3;
file_name = '160721_anette_egg_sponge_3-1.txt';
InverseKinematicBenchmarkPlayback( p0, 14, 'Projected Gauss Seidel SOR with Limits', file_name, 'relative', 2);
InverseKinematicBenchmarkPlayback( p0, 14, 'Gauss Seidel SOR', file_name, 'relative', 2);
InverseKinematicBenchmarkPlayback( p0, 14, 'SVD', file_name, 'relative', 2);
InverseKinematicBenchmarkPlayback( p0, 14, 'Damped Least Square', file_name, 'relative', 2);

end
