sam_per_read = 500;
sample_times = 5;
sam_per_read = 10;
Fs = 200;

if ~exist("ESP_obj","var")
    disp("Connecting to ESP32");
    ESP_obj = arduino("COM7",'ESP32-WROOM-DevKitC',"Libraries","I2C");
end
clear imu_obj;
scanI2CBus(ESP_obj)
imu_obj = mpu6050(ESP_obj,'OutputFormat','matrix','TimeFormat','duration','I2CAddress',0x68,SampleRate=Fs,SamplesPerRead=sam_per_read);
disp("MPU Connected");

fuse = complementaryFilter("SampleRate",200,"HasMagnetometer",false,"OrientationFormat","quaternion");
accel_all = zeros(sam_per_read*sample_times,3);
gyro_all = zeros(sam_per_read*sample_times,3);
for i=0:sample_times-1
    fprintf("Sampling%d\n",i);
    [accel, gyro, ~, ~] = imu_obj.read;
    accel_all(i*sam_per_read+1:(i+1)*sam_per_read,:) = accel;
    gyro_all(i*sam_per_read+1:(i+1)*sam_per_read,:) = gyro;
end
filename = strcat(string(datetime('now','Format','uuuu-MM-dd-HH-mm-ss')),'.mat');
save(filename,"accel_all","gyro_all");

[orientation_q,~] = fuse(accel_all,gyro_all);
eulerAngles = euler(orientation_q,'ZYX','point');
eulerAngles = rad2deg(eulerAngles);
yaw = eulerAngles(:,1);
pitch = eulerAngles(:,2);
roll = eulerAngles(:,3);
[avarYaw,~] = allanvar(yaw,'octave',Fs);
[avarPitch,~] = allanvar(pitch,'octave',Fs);
[avarRoll,tau] = allanvar(roll,'octave',Fs);
tbl = table(tau, avarYaw, avarPitch, avarRoll);
loglog(tbl, "tau", ["avarYaw" "avarPitch" "avarRoll"]);

% [avar,tau] = allanvar(accel_all,'octave',Fs);
% axAvar = avar(:,1);
% ayAvar = avar(:,2);
% azAvar = avar(:,3);
% tbl = table(tau, axAvar, ayAvar, azAvar);
% loglog(tbl, "tau", ["axAvar" "ayAvar" "azAvar"]);

grid on;
legend;
xlabel('\tau');
ylabel('\sigma^2(\tau)');
title('Allan Variance');