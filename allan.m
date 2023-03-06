sam_per_read = 500;
Fs = 200;

if isempty(ESP_obj)
    ESP_obj = arduino("COM7",'ESP32-WROOM-DevKitC',"Libraries","I2C");
    scanI2CBus(ESP_obj)
    imu_obj = mpu6050(ESP_obj,'OutputFormat','matrix','TimeFormat','duration','I2CAddress',0x68,SampleRate=Fs,SamplesPerRead=sam_per_read);
end
if isempty(imu_obj)==false
    imu_obj.release
end
    imu_obj = mpu6050(ESP_obj,'OutputFormat','matrix','TimeFormat','duration','I2CAddress',0x68,SampleRate=Fs,SamplesPerRead=sam_per_read);

[accel, ~, ~, ~] = imu_obj.read;
[avar,tau] = allanvar(accel,Fs);
ax_avar = avar(:,1);
ay_avar = avar(:,2);
az_avar = avar(:,3);

tbl = table(tau,ax_avar,ay_avar,az_avar);
loglog(tbl,"tbl",["ax_avar" "ay_avar" "az_avar"]);
grid on;
legend;
xlabel('\tau');
ylabel('\sigma^2(\tau)');
title('Allan Variance');