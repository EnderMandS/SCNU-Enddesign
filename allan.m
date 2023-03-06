sam_per_read = 500;
sample_times = 4;
Fs = 200;

if ~exist("ESP_obj","var")
    disp("Connecting to ESP32");
    ESP_obj = arduino("COM7",'ESP32-WROOM-DevKitC',"Libraries","I2C");
end
clear imu_obj;
scanI2CBus(ESP_obj)
imu_obj = mpu6050(ESP_obj,'OutputFormat','matrix','TimeFormat','duration','I2CAddress',0x68,SampleRate=Fs,SamplesPerRead=sam_per_read);
disp("MPU Connected");

accel_all = zeros(sam_per_read*sample_times,3);
for i=0:sample_times-1
    disp("Sampling");
    [accel, ~, ~, ~] = imu_obj.read;
    accel_all(i*sam_per_read+1:(i+1)*sam_per_read,:) = accel;
end
[avar,tau] = allanvar(accel_all,'octave',Fs);
axAvar = avar(:,1);
ayAvar = avar(:,2);
azAvar = avar(:,3);

tbl = table(tau, axAvar, ayAvar, azAvar);
head(tbl,3)
loglog(tbl, "tau", ["axAvar" "ayAvar" "azAvar"]);
grid on;
legend;
xlabel('\tau');
ylabel('\sigma^2(\tau)');
title('Allan Variance');