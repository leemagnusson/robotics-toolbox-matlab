%function [ Pm_return ] = estimate_log_power(r, other_param, varargin)

    %Pm_return = zeros(2, length(varargin))

    %for lognum = 1:length(varargin)
        %logname = varargin{lognum}

        data = import_log(logname);
        ws = [-1,1,-1,1,-1,1];

        t_tmp = data(:,2)-data(1,2);
        switch logname
            case 'logs/hernia.log'
                data = data(t_tmp>34,:);
                ws = [-.5,.5,-.1,.8,-.1,.7];
            case 'logs/lowerAnteria.log';
                ws = [-.4,.4,-.5,.5,-.3,.8];
        end

        %%
        
        % Get joint position data from log file (where is this
        % documented?)
        qm = data(:,[129,134,89,94,139,144,119,154,149]);

        tm = data(:,2)-data(1,2);
        dt = 1/3000;
        t = 0:dt:max(tm);
        q = interp1(tm,qm,t,'spline');

        %qs = idealfilt(t,q,1000,'lowpass',2);
        %qs = idealfilter(timeseries(q,t),[0,1000],'pass')
        %q = qs.Data
        Wn = 10/(3000/2)
        [b,a] = butter(4, Wn, 'low');
        freqz(b,a,1000000,3000)
        
        qs = filtfilt(b, a, q);
        
        %FFTq = fft(q);
        %FFTqs = fft(qs);
        %f = (1/dt) * (1:(size(q,1)))/size(q,1);
                
        %figure(); hold all;
        %plot(f, FFTq(:,7));
        %figure();
        %plot(f, FFTqs(:,7));
        
        figure();
        plot(t,q);
        title('q');
        figure();
        plot(t,qs);
        title('qs');
        
        qd = [zeros(1,size(qs,2));diff(qs,1,1)]/dt;
        qdd = [zeros(1,size(qs,2));diff(qs,2,1);zeros(1,size(qs,2))]/dt^2;
        
        figure();
        plot(t, qd);
        title('qd');
        
        figure();
        plot(t, qdd);
        title('qdd');
        
%         return;
%         
%         qd = zeros(size(q));
%         qdd = zeros(size(q));
%         for i = 1:size(qm, 2)
%             qd(:,i) = movingslope(q(:,i),300,3,dt);
%             qdd(:,i) = movingslope(qd(:,i),150,3,dt);
%         end



        %return

        %% Inverse dynamics 

        T = r.rne(qs,qd,qdd);

        figure(); clf;
        plot(t,T);
        grid on;
        legend(other_param.name1,'location','best');
        accel = max(abs(T),[],1) - abs(mean(T))
        title(strcat(logname, ' joint torque'));

        
         %%

        [Pm,Pt,Tm,Tm_clipped, Pm_clipped] = motor_power_heat(T,qd,other_param);
        Pm_sum = sum(Pm_clipped,2);
        figure(); clf; hold on;
        plot(t,Pm_sum,'c-.', t,Pm_clipped);
        legend(['Power sum', other_param.name1'], 'location','best');
        title(strcat(logname, ' motor power'));
        
        Pm_return(:,lognum) = [max(Pm_sum), sqrt(mean(Pm_sum.^2))];

    %end
%end