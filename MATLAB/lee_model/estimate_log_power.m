function [ results, all_powers ] = estimate_log_power( r, other_param, gravities, outfile_prefix, varargin)
% Estimates power usage for several logs returning aggregate results and
% saving plots for every arm in every log.
%
% Parameters:
% r                 The robot
% other_params      Other parameters structure imported with the robot
% gravities         Gravity column vectors to run the test in.
% outfile_prefix    Prefix to put on plots saved for each log/arm/gravity
% varargin          List of log files to process
  
    logfilenames = {};
    armnumbers = [];
    max_powers = [];
    rms_powers = [];
    gravity_used = [];
    

    lowpass_cut = 15;
    dt = 1/3000;

    all_powers = [];

    for lognum = 1:length(varargin)
        logname = varargin{lognum}{1}
        class(logname)
        [logpath, logfile, logext] = fileparts(logname);
        
        % Run on all 4 arms (skipping any arm that doesn't move)
        for armnum = 0:3
        
            % Import joint positions from logfile:
            [t, q, eef, ws] = import_logfile(logname, 'arm', armnum, 'pitch1', 'interp_dt', dt, 'spline', 'smooth', lowpass_cut);

            for gravnum = 1:size(gravities, 2)
                gravity = gravities(:,gravnum);
                
                r.gravity = gravity;
        
                disp([datestr(datetime('now')), ': Processing ', logname, ' arm ', num2str(armnum), ', gravity ', num2str(gravity'), '.']);


                %%

                fig = figure('units', 'normalized', 'outerposition',[0 0 1 1]);

                subplot(3, 3, 1);
                plot(t,q);
                title([logfile, ' arm ', num2str(armnum), ' q filtered']);

                qd = [zeros(1,size(q,2));diff(q,1,1)]/dt;
                qdd = [zeros(1,size(q,2));diff(q,2,1);zeros(1,size(q,2))]/dt^2;

                subplot(3, 3, 2);
                plot(t, qd);
                title('qd');

                subplot(3, 3, 3);
                plot(t, qdd);
                title('qdd');




        %         Alternative way to get derivative using moving slope
        %         qd = zeros(size(q));
        %         qdd = zeros(size(q));
        %         for i = 1:size(qm, 2)
        %             qd(:,i) = movingslope(q(:,i),300,3,dt);
        %             qdd(:,i) = movingslope(qd(:,i),150,3,dt);
        %         end

                %% Inverse dynamics 

                % Don't bother with dynamics if we don't move at all (i.e
                % values in qd are 0)
                if ~any(any(qd))
                    T = repmat(r.gravload(q(1,:)), size(q,1), 1); % Not moving, so torques are all gravity, and the same
                else            
                    T = r.rne(q,qd,qdd); % Inverse dynamics
                end

                subplot(3, 3, 4);
                plot(t,T);
                grid on;
                leg = legend(other_param.name1,'location','best');
                set(leg, 'visible', 'off');
                accel = max(abs(T),[],1) - abs(mean(T));
                title(strcat(logfile, ' arm', num2str(armnum), ' joint torque'));


                 %%

                [Pm,Pt,Tm,Im] = motor_torque_speed_pwr(T,qd,other_param);
                Pm_sum = sum(Pm,2);
                subplot(3, 3, 5); hold on;
                plot(t,Pm_sum,'c-.', t,Pm);
                leg = legend(['Power sum', other_param.name1'], 'location','best');
                set(leg, 'visible', 'off');
                title(strcat(logfile, ' motor power'));

                subplot(3, 3, 6);
                histogram(Pm_sum);
                set(gca, 'YScale', 'log');
                title('Mot power histogram (log scale)');

                subplot(3, 3, 7);
                plot(t,Tm);
                title('Motor torque');

                subplot(3, 3, 8);
                plot(t,Im);
                title('Motor current');

                subplot(3, 3, 9);
                %r.plot3d(q(1,:), 'path', [robot_dir, '/stl'], 'workspace',[-1,1,-.5,1.25,-.1,1],'jaxes' );
                %hold on;
                quiver3(0, 0, 0, gravity(1), gravity(2), gravity(3), '-r', 'LineWidth', 2);
                axis([-10, 10, -10, 10, -10, 10]);
                xlabel('X');
                ylabel('Y');
                zlabel('Z');
                grid on;
                title(['Gravity [', num2str(gravity'), ']']);

                Pm_return(:,lognum) = [max(Pm_sum), sqrt(mean(Pm_sum.^2))];

                %%
                %Save results
                logfilenames(length(logfilenames)+1, 1) = {logname};
                armnumbers = [armnumbers; armnum];
                max_powers = [max_powers; max(Pm_sum)];
                rms_powers = [rms_powers; sqrt(mean(Pm_sum.^2))];
                gravity_used = [gravity_used; gravity'];

                saveas(fig.Number,[outfile_prefix, logfile, '_arm', num2str(armnum), '_grav', num2str(gravnum) '.pdf'],'pdf');

                all_powers = vertcat(all_powers, Pm_sum);
            end
        end
    end
    
    results = table(logfilenames,armnumbers,max_powers,rms_powers,gravity_used)
end