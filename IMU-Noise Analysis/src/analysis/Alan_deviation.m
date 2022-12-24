M = readmatrix("imu_correct.csv");
L=length(M);
axis1 = ['x','y','z'];
data_freq=40;
% Calculate allan variances of Gyroscope along x,y,z
omegax=M(:,15);
omegay=M(:,16);
omegaz=M(:,17);

[avarx,taux] = alan_var(omegax,data_freq);
[avary,tauy] = alan_var(omegay,data_freq);
[avarz,tauz] = alan_var(omegaz,data_freq);

adev_gyro = [sqrt(avarx),sqrt(avary),sqrt(avarz)];
tau_gyro = [taux,tauy,tauz];

% Find angle random walk
Nslope = -0.5;
N_gyro=[1,1,1];
len_gyro = length(tau_gyro(:,1));
lineN_gyro=ones(len_gyro,3);
tauN_gyro=1;
for i = [1,2,3]
    logtau = log10(tau_gyro(:,i));
    logadev = log10(adev_gyro(:,i));
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, y] = min(abs(dlogadev - Nslope));
    b_arw = logadev(y) - Nslope*logtau(y);

    % Determine the angle random walk coefficient
    logN = Nslope*log(1) + b_arw;
    N_gyro(i) = 10^logN;
    % Generate line for angle random walk
    lineN_gyro(:,i)=N_gyro(i)./sqrt(tau_gyro(:,i));
end

% Find Bias Instability
Bslope = 0;
B_gyro=[1,1,1];
len_gyro = length(tau_gyro(:,1));
lineB_gyro=ones(len_gyro,3);
scfB = sqrt(2*log(2)/pi);
tauB_gyro=[1,1,1];
for i = [1,2,3]
    logtau = log10(tau_gyro(:,i));
    logadev = log10(adev_gyro(:,i));
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, y] = min(abs(dlogadev - Bslope));
    b_bi = logadev(y) - Bslope*logtau(y);

    % Determine the angle random walk coefficient
    logB = b_bi - log10(scfB);
    B_gyro(i) = 10^logB;
    lineB_gyro(:,i) = B_gyro(i) * scfB * ones(size(tau_gyro(:,i)));
    tauB_gyro(i)=tau_gyro(y,i);
end

% Find rate random walk
Kslope = 0.5;
K_gyro=[1,1,1];
len_gyro = length(tau_gyro(:,1));
lineK_gyro=ones(len_gyro,3);
tauK_gyro=3;
for i = [1,2,3]
    logtau = log10(tau_gyro(:,i));
    logadev = log10(adev_gyro(:,i));
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, y] = min(abs(dlogadev - Kslope));
    b_rrw = logadev(y) - Kslope*logtau(y);

    % Determine the angle random walk coefficient
    logK = Kslope*log10(3) + b_rrw;
    K_gyro(i) = 10^logK;
    % Generate line for angle random walk
    lineK_gyro(:,i)=K_gyro(i).*sqrt(tau_gyro(:,i)/3);
end

% Plot the allan deviation graph for Gyro
figure(1)
sgtitle('Gyroscope-Allan Deviation with Angle Random Walk and Bias Instability')
for i = [1,2,3]
    subplot(3,1,i)
    loglog(tau_gyro(:,i),adev_gyro(:,i), ...
        tau_gyro(:,i), lineN_gyro(:,i), '--', tauN_gyro, N_gyro(i), 'x', ...
        tau_gyro(:,i), lineB_gyro(:,i), '--', tauB_gyro(i), scfB*B_gyro(i), 'o', ...
        tau_gyro(:,i), lineK_gyro(:,i), '--', tauK_gyro, K_gyro(i), 'x')
    xlabel('\tau')
    ylabel('\sigma(\tau)')
    legend('$\sigma (rad/s)$', ...
        '$\sigma_N ((rad/s)/\sqrt{Hz})$','', ...
        '$\sigma_B (rad/s)$', ...
        'Interpreter', 'latex')
    grid on
    N_gyrodeg(i)=N_gyro(i)*(180/pi());
    B_gyrodeg(i)=B_gyro(i)*(180/pi());
    K_gyrodeg(i)=K_gyro(i)*(180/pi());
    t = sprintf('Axis: %s ,Bias Instability: %f deg/sec, Noise Density: %f deg/sec/sqrt(Hz), RRW:%f',axis1(i),B_gyrodeg(i),N_gyrodeg(i),K_gyrodeg(i));
    title(t)
end



alphax=M(:,19);
alphay=M(:,20);
alphaz=M(:,21);
[avar_ax,tau_ax] = alan_var(alphax,data_freq);
[avar_ay,tau_ay] = alan_var(alphay,data_freq);
[avar_az,tau_az] = alan_var(alphaz,data_freq);

adev_acc = [sqrt(avar_ax),sqrt(avar_ay),sqrt(avar_az)];
tau_acc = [tau_ax,tau_ay,tau_az];

% Find angle random walk
slope = -0.5;
N_acc=[1,1,1];
len_acc = length(tau_acc(:,1));
lineN_acc=ones(len_acc,3);
tauN_acc=1;
for i = [1,2,3]
    logtau = log10(tau_acc(:,i));
    logadev = log10(adev_acc(:,i));
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, y] = min(abs(dlogadev - slope));
    b_arw = logadev(y) - slope*logtau(y);

    % Determine the angle random walk coefficient
    logN = slope*log(1) + b_arw;
    N_acc(i) = 10^logN;
    % Generate line for angle random walk
    lineN_acc(:,i)=N_acc(i)./sqrt(tau_acc(:,i));
end

% Find Bias Instability
slope = 0;
B_acc=[1,1,1];
lineB_acc=ones(len_acc,3);
scfB = sqrt(2*log(2)/pi);
tauB_acc=[1,1,1];
for i = [1,2,3]
    logtau = log10(tau_acc(:,i));
    logadev = log10(adev_acc(:,i));
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, y] = min(abs(dlogadev - slope));
    b_bi = logadev(y) - slope*logtau(y);

    % Determine the angle random walk coefficient
    logB = b_bi - log10(scfB);
    B_acc(i) = 10^logB;
    lineB_acc(:,i) = B_acc(i) * scfB * ones(size(tau_acc(:,i)));
    tauB_acc(i)=tau_acc(y,i);
end

% Find rate random walk
slope = 0.5;
K_acc=[1,1,1];
len_acc = length(tau_acc(:,1));
lineK_acc=ones(len_acc,3);
tauK_acc=3;
for i = [1,2,3]
    logtau = log10(tau_acc(:,i));
    logadev = log10(adev_acc(:,i));
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, y] = min(abs(dlogadev - slope));
    b_rrw = logadev(y) - slope*logtau(y);

    % Determine the angle random walk coefficient
    logK = slope*log(3) + b_rrw;
    K_acc(i) = 10^logK;
    % Generate line for angle random walk
    lineK_acc(:,i)=K_acc(i).*sqrt(tau_acc(:,i)/3);
end

% Plot the allan deviation graph for Accelerometer
figure(2)
sgtitle('Accelerometer-Allan Deviation with Angle Random Walk and Bias Instability')
for i = [1,2,3]
    subplot(3,1,i)
    loglog(tau_acc(:,i),adev_acc(:,i), ...
        tau_acc(:,i), lineN_acc(:,i), '--', tauN_acc, N_acc(i), 'x', ...
        tau_acc(:,i), lineB_acc(:,i), '--', tauB_acc(i), scfB*B_acc(i), 'o', ...
        tau_acc(:,i), lineK_acc(:,i), '--', tauK_acc, K_acc(i), 'x')
    xlabel('\tau')
    ylabel('\sigma(\tau)')
    legend('$\sigma (m/s/s)$', ...
        '$\sigma_N ((m/s/s)/\sqrt{Hz})$','', ...
        '$\sigma_B (m/s/s)$','', ...
        'Interpreter', 'latex')
    grid on
    N_acc(i)=N_acc(i)*(100);
    B_acc(i)=B_acc(i)*(100);
    title(sprintf('Axis: %s ,In-run Bias Stability: %f mg, Noise Density: %f mg/sqrt(Hz) RRW: %f', axis1(i),B_acc(i),N_acc(i),K_acc(i)))
    subtitle('(1 m/sec^2 = 100 mg)')
end

%modeling imu_sensor
data_freq = 40;
Nsamples = L;
t=0:1/data_freq:(Nsamples-1)/data_freq;
params_gyro = gyroparams('BiasInstability',B_gyro(2),'NoiseDensity',N_gyro(2),'RandomWalk',K_gyro(2));
params_acc = accelparams('BiasInstability',B_acc(2),'NoiseDensity',N_acc(2),'RandomWalk',K_gyro(2));
imu = imuSensor("accel-gyro",'SampleRate',data_freq,"Gyroscope",params_gyro, "Accelerometer",params_acc);
orient = quaternion.ones(Nsamples, 1);
acc = zeros(Nsamples, 3);
angvel = zeros(Nsamples, 3);
[~, gyroData] = imu(acc, angvel, orient);
[avarSim,tauSim] = allanvar(gyroData,'octave',data_freq);
adevSim = sqrt(avarSim);
adevSim = mean(adevSim,2);

%Plot modeled imu Allan Deviation
figure(3)
loglog(tau_gyro,adev_gyro(:,2),tauSim,adevSim)
title('Allan Deviation of Measured and Simulated Gyroscope')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('Measured', 'Simulated')
grid on
axis equal;

figure(4)
loglog(tau_gyro(:,2),adev_gyro(:,2), ...
       tau_gyro(:,2), lineN_gyro(:,2), '--', tauN_gyro, N_gyro(2), 'x', ...
        tau_gyro(:,2), lineB_gyro(:,2), '--', tauB_gyro(2), scfB*B_gyro(2), 'o', ...
        tau_gyro(:,2), lineK_gyro(:,2), '--', tauK_gyro, K_gyro(2), 'x')
title('Allan Deviation Y-Axis')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('$\sigma (rad/s)$', ...
        '$\sigma_N ((rad/s)/\sqrt{Hz})$','', ...
        '$\sigma_B (rad/s)$','', ...
        '$\sigma_K ((rad/s)/\sqrt{Hz})$',...
        'Interpreter', 'latex')
grid on
axis equal;


function [avar,tau]=alan_var(data,freq)
    t0 = 1/freq;
    theta = cumsum(data, 1)*t0;
    maxNumM = 100;
    L = size(theta, 1);
    maxM = 2.^floor(log2(L/2));
    m = logspace(log10(1), log10(maxM), maxNumM).';
    m = ceil(m); % m must be an integer.
    m = unique(m); % Remove duplicates.

    tau = m*t0;

    avar = zeros(numel(m), 1);
    for i = 1:numel(m)
        mi = m(i);
        avar(i,:) = sum( ...
            (theta(1+2*mi:L) - 2*theta(1+mi:L-mi) + theta(1:L-2*mi)).^2, 1);
    end
    avar = avar ./ (2*tau.^2 .* (L - 2*m));
end