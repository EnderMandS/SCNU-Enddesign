% 从文件中读取IMU数据分析
clear;

disp('Loading file');
% TODO
load( strcat('data/','2023-03-20-23-13-06','.mat') );

time_len = length(time);
fs = floor(time_len / (time(time_len)-time(1)));
fft_len = (-time_len/2:time_len/2-1)*(fs/time_len);

for i = time_len:1  % time从0开始
    time(i) = time(i)-time(1);
end

% 加速度
figure('Name','Accleration');
title('Accleration');
xlabel('t/s');
ylabel('m^2/s');
plot(time,accel(:,1), time,accel(:,2), time,accel(:,3));
grid on;
legend('ax','ay','az');

% FFT
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
fuse = complementaryFilter("SampleRate",fs,"HasMagnetometer",false,"OrientationFormat","quaternion");
[orientation_q,~] = fuse(accel,gyro);
eulerAngles = rad2deg(euler(orientation_q,'ZYX','point'));
figure('Name', 'Eular Angle');
title('Eular Angle');
xlabel("t/s");
ylabel("degree");
plot(time,eulerAngles(:,1), time,eulerAngles(:,2), time,eulerAngles(:,3));
grid on;
legend('yaw','pitch','roll');

% Allan方差分析
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
