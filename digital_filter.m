% 从文件中读取IMU数据分析并滤波
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

% FFT
disp('Calculating FFT');
ax_fft = fftshift(fft(accel(:,1)));
ay_fft = fftshift(fft(accel(:,2)));
az_fft = fftshift(fft(accel(:,3)));
ax_power = abs(ax_fft).^2/time_len;
ay_power = abs(ay_fft).^2/time_len;
az_power = abs(az_fft).^2/time_len;
figure('Name', 'FFT before filter');
title('FFT before filter');
xlabel("f (Hz)");
ylabel("|P(f)|");
plot(fft_len,ax_power, fft_len,ay_power, fft_len,az_power);
grid on;
legend('ax','ay','az');
 
for i=fix(length(fft_len)/2):length(fft_len)
    if fft_len(i)>5     %搜索起始频率
        start_index = i;
        break
    end
end
[power_max,power_max_i] = max(cat(1,ax_power(start_index:end), ...
    ay_power(start_index:end),az_power(start_index:end)));
power_threshold = 0.2;      %搜索阈值
in_stop = false;
stop_low = []; stop_up = [];
for i=start_index:length(fft_len)
    if max([ax_power(i) ay_power(i) az_power(i)])>power_max*power_threshold && ~in_stop
            stop_low = [stop_low,fft_len(i)];
            in_stop = true;
    end
    if  max([ax_power(i) ay_power(i) az_power(i)])<power_max*power_threshold && in_stop
            stop_up = [stop_up,fft_len(i)];
            in_stop = false;
    end
end
if isempty(stop_low) || isempty(stop_up) || ...
    length(stop_low)~=length(stop_up)
    error('stop detect fail');
end
if length(stop_low)>1
    cnt = 1;
    stop_low_com(1) = stop_low(1);
    for i=1:length(stop_low)-1
        if stop_low(i+1)-stop_up(i)>3
            stop_up_com(cnt) = stop_up(i);
            cnt = cnt+1;
            stop_low_com(cnt) = stop_low(i+1);
        end
    end
    stop_up_com(cnt) = stop_up(end);
else
    stop_low_com = stop_low(1);
    stop_up_com = stop_up(1);
end

disp(['Find ', num2str(length(stop_low_com)), ' stop band']);
N = [];
Wn = [];
a = [];
b = [];
for i=1:length(stop_low_com)
    fsl = stop_low_com(i); fsu = stop_up_com(i);
    fpl = fsl-5; fpu = fsu+5;
    ws = [2*fsl/fs, 2*fsu/fs];  %边界频率归一化
    wp = [2*fpl/fs, 2*fpu/fs];
    rp = 1;                     %幅度失真dB
    rs = 40;                    %衰减dB
    [N_t, Wn_t] = ellipord(wp,ws,rp,rs)
    [b_t,a_t] = ellip(N_t,rp,rs,Wn_t,'stop');
    name = join(['Frequency response ', num2str(i)]);
    figure('Name', name);
    title(name);
    freqz(b_t,a_t)
    accel = filter(b_t,a_t,accel);
    N = [N,N_t]; Wn = [Wn,Wn_t]; b = [b,b_t]; a = [a,a_t];
end


% fsl = 19; fsu = 25;         %滤除频段
% fpl = fsl-5; fpu = fsu+5;   %保留频段
% ws = [2*fsl/fs, 2*fsu/fs];  %边界频率归一化
% wp = [2*fpl/fs, 2*fpu/fs];
% rp = 1;                     %幅度失真dB
% rs = 40;                    %衰减dB
% [N, Wn] = ellipord(wp,ws,rp,rs)
% [b,a] = ellip(N,rp,rs,Wn,'stop');
% figure('Name', 'Frequency response');
% title('Frequency response');
% freqz(b,a)
% disp('Calculating accleration');
% accel = filter(b,a,accel);

% FFT
disp('Calculating FFT');
ax_fft = fftshift(fft(accel(:,1)));
ay_fft = fftshift(fft(accel(:,2)));
az_fft = fftshift(fft(accel(:,3)));
ax_power = abs(ax_fft).^2/time_len;
ay_power = abs(ay_fft).^2/time_len;
az_power = abs(az_fft).^2/time_len;
figure('Name', 'FFT after filter');
title('FFT after filter');
xlabel("f (Hz)");
ylabel("|P(f)|");
plot(fft_len,ax_power, fft_len,ay_power, fft_len,az_power);
grid on;
legend('ax','ay','az');

figure('Name','Accleration after filter');
title('Accleration after filter');
xlabel('t/s');
ylabel('m^2/s');
plot(time,accel(:,1), time,accel(:,2), time,accel(:,3));
grid on;
legend('ax','ay','az');

% 互补对称滤波Euler角
disp('Calculating complementary eular angle');
addpath('quaternion_library');
AHRS = MahonyAHRS('SamplePeriod',1/fs, 'Kp',0.5, 'Ki',0.01);
orientation_q = zeros(time_len, 4);
for i = 1:time_len
    AHRS.UpdateIMU(gyro(i,:), accel(i,:)/9.7883);	% gyroscope units must be radians
    orientation_q(i,:) = AHRS.Quaternion;
end
orientation_q = quaternion(quaternConj(orientation_q));
eulerAngles = rad2deg(euler(orientation_q,'ZYX','point'));
figure('Name', 'Complementary filter eular angle after filter');
title('Complementary filter eular angle after filter');
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
figure('Name', 'EKF eular angle after filter');
title('EKF eular angle after filter');
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
figure('Name','Allan Variance after filter');
title('Allan Variance after filter');
loglog(tbl, "tau", ["axAvar" "ayAvar" "azAvar"]);
grid on;
legend;
xlabel('\tau');
ylabel('\sigma^2(\tau)');
