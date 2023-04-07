% 从文件中读取IMU数据分析
clear;

disp('Loading file');
load( strcat('data/','2023-04-07-15-56-18','.mat') );

time_len = length(time);
fs = floor(time_len / (time(time_len)-time(1)));
fft_len = (-time_len/2:time_len/2-1)*(fs/time_len);

start_time = time(1);
for i = 1:time_len  % time从0开始
    time(i) = time(i)-start_time;
end

% 加速度
disp('Calculating accleration');
figure('Name','Accleration');
title('Accleration');
xlabel('t/s');
ylabel('m^2/s');
plot(time,accel(:,1), time,accel(:,2), time,accel(:,3));
grid on;
legend('ax','ay','az');

% 陀螺仪
disp('Calculating gyroscope');
figure('Name','Gyro');
title('Gyroscope');
xlabel('t/s');
ylabel('rad/s');
plot(time,gyro(:,1), time,gyro(:,2), time,gyro(:,3));
grid on;
legend('gx','gy','gz');

% FFT
disp('Calculating FFT');
ax_fft = fftshift(fft(accel(:,1)));
ay_fft = fftshift(fft(accel(:,2)));
az_fft = fftshift(fft(accel(:,3)));
ax_power = abs(ax_fft).^2/time_len;
ay_power = abs(ay_fft).^2/time_len;
az_power = abs(az_fft).^2/time_len;
figure('Name', 'FFT');
title('FFT');
xlabel("f (Hz)");
ylabel("|P(f)|");
plot(fft_len,ax_power, fft_len,ay_power, fft_len,az_power);
grid on;
legend('ax','ay','az');

% 互补对称滤波Euler角
disp('Calculating complementary eular angle');

% comFilter = complementaryFilter("SampleRate",fs,"HasMagnetometer",false, ... 
%     "OrientationFormat","quaternion","AccelerometerGain",0.8);
% [orientation_q,~] = comFilter(accel,gyro);

addpath('quaternion_library');
AHRS = MahonyAHRS('SamplePeriod',1/fs, 'Kp',0.5, 'Ki',0.01);
orientation_q = zeros(time_len, 4);
for i = 1:time_len
    AHRS.UpdateIMU(gyro(i,:), accel(i,:)/9.7883);	% gyroscope units must be radians
    orientation_q(i,:) = AHRS.Quaternion;
end
orientation_q = quaternion(quaternConj(orientation_q));

eulerAngles = rad2deg(euler(orientation_q,'ZYX','point'));
figure('Name', 'Complementary filter eular angle');
title('Complementary filter eular angle');
xlabel("t/s");
ylabel("degree");
plot(time,eulerAngles(:,1), time,eulerAngles(:,2), time,eulerAngles(:,3));
grid on;
legend('yaw','pitch','roll');

% EKF
disp('Calculating EKF eular angle');
clear eulerAngles;
clear orientation_q;
ins_accel = insAccelerometer;
ins_gyro = insGyroscope;
filter_EKF = insEKF(ins_accel,ins_gyro);
stateparts(filter_EKF,"Orientation",[1 0 0 0]);
statecovparts(filter_EKF,"Orientation",1e-2);
accNoise = 0.01;
gyroNoise = 0.01;
processNoise = diag([0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1]);
filter_EKF.AdditiveProcessNoise = processNoise;
for i = 1:length(time)
    fuse(filter_EKF,ins_accel,accel(i,:),diag([0.1 0.1 0.1]));
    fuse(filter_EKF,ins_gyro,gyro(i,:),diag([0.2 0.2 0.2]));
    orientation_q(i) = quaternion(stateparts(filter_EKF,"Orientation"));
end
eulerAngles = rad2deg(euler(orientation_q,'ZYX','point'));
figure('Name', 'EKF eular angle');
title('EKF eular angle');
xlabel("t/s");
ylabel("degree");
plot(time,eulerAngles(:,1), time,eulerAngles(:,2), time,eulerAngles(:,3));
grid on;
legend('yaw','pitch','roll');

% Allan方差分析
disp('Calculating allanvar');
[avar,tau] = allanvar(accel,1:floor((time_len-1)/2),fs);
axAvar = avar(:,1);
ayAvar = avar(:,2);
azAvar = avar(:,3);
tbl = table(tau, axAvar, ayAvar, azAvar);
figure('Name','Allan Variance');
loglog(tbl, "tau", ["axAvar" "ayAvar" "azAvar"]);
grid on;
legend;
xlabel('\tau');
ylabel('\sigma^2(\tau)');
title('Allan Variance');
