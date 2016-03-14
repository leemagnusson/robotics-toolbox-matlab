%function [ Pm_return ] = estimate_log_power(r, other_param, varargin)

    %Pm_return = zeros(2, length(varargin))

    %for lognum = 1:length(varargin)
        %logname = varargin{lognum}

        data = import_log(logname);


        %%
        
        % Get joint position data from log file (where is this
        % documented?)
        qm = data(:,[129,134,89,94,139,144,119,154,149]);

        % Freq to filter q (in Hz)
        lowpass_cut = 10;
        tm = data(:,2)-data(1,2);
        dt = 1/3000;
        t = 0:dt:max(tm);
        q = interp1(tm,qm,t,'spline');

        %qs = idealfilt(t,q,1000,'lowpass',2);
        %qs = idealfilter(timeseries(q,t),[0,1000],'pass')
        %q = qs.Data
        Wn = lowpass_cut/((1/dt)/2)
        [b,a] = butter(4, Wn, 'low');
        %freqz(b,a,1000000,3000)
        
        qs = filtfilt(b, a, q);
        
        figure(lognum);
      
        subplot(2, 3, 1);
        plot(t,qs);
        title([logname, ' q filtered']);
        
        qd = [zeros(1,size(qs,2));diff(qs,1,1)]/dt;
        qdd = [zeros(1,size(qs,2));diff(qs,2,1);zeros(1,size(qs,2))]/dt^2;
        
        subplot(2, 3, 2);
        plot(t, qd);
        title('qd');
        
        subplot(2, 3, 3);
        plot(t, qdd);
        title('qdd');
        
%         Alternative way to get derivative using moving slope
%         qd = zeros(size(q));
%         qdd = zeros(size(q));
%         for i = 1:size(qm, 2)
%             qd(:,i) = movingslope(q(:,i),300,3,dt);
%             qdd(:,i) = movingslope(qd(:,i),150,3,dt);
%         end



        %return

        %% Inverse dynamics 

        T = r.rne(qs,qd,qdd);

        subplot(2, 3, 4);
        plot(t,T);
        grid on;
        legend(other_param.name1,'location','best');
        accel = max(abs(T),[],1) - abs(mean(T))
        title(strcat(logname, ' joint torque'));

        
         %%

        [Pm,Pt,Tm,Im] = motor_torque_speed_pwr(T,qd,other_param);
        Pm_sum = sum(Pm,2);
        subplot(2, 3, 5); hold on;
        plot(t,Pm_sum,'c-.', t,Pm);
        legend(['Power sum', other_param.name1'], 'location','best');
        title(strcat(logname, ' motor power'));
        
        subplot(2, 3, 6);
        histogram(Pm_sum);
        set(gca, 'YScale', 'log');
        title('Mot power histogram (log scale)');
        
        Pm_return(:,lognum) = [max(Pm_sum), sqrt(mean(Pm_sum.^2))];

    %end
%end