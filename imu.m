% 参数定义
global buffer_available;
global ax_data;
global ay_data;
global az_data;
sam_per_read = 10;
buffer_length = 1000;
fs = 200;

% 创建数据缓冲区
ax_data = zeros(1,buffer_length);
ay_data = zeros(1,buffer_length);
az_data = zeros(1,buffer_length);

% 初始化硬件
if isempty(ESP_obj)
    ESP_obj = arduino("COM7",'ESP32-WROOM-DevKitC',"Libraries","I2C");
    scanI2CBus(ESP_obj)
    imu_obj = mpu6050(ESP_obj,'OutputFormat','matrix','TimeFormat','duration','I2CAddress',0x68,SampleRate=fs,SamplesPerRead=sam_per_read);
end
if isempty(imu_obj)
    imu_obj = mpu6050(ESP_obj,'OutputFormat','matrix','TimeFormat','duration','I2CAddress',0x68,SampleRate=fs,SamplesPerRead=sam_per_read);
end

% 定义动态绘图结构体
an_ax = animatedline('MaximumNumPoints',2000,'Color',"#0072BD");
an_ay = animatedline('MaximumNumPoints',2000,'Color',"#D95319");
an_az = animatedline('MaximumNumPoints',2000,'Color',"#EDB120");
% an_gx = animatedline('MaximumNumPoints',2000,'Color',"#7E2F8E");
% an_gy = animatedline('MaximumNumPoints',2000,'Color',"#4DBEEE");
% an_gz = animatedline('MaximumNumPoints',2000,'Color',"#A2142F");

data_cnt = 0;
while 1
    [accel, gyro, timeStamps, overrun] = imu_obj.read;

    % 数据转换
    duration_ms = milliseconds(timeStamps);
    ax_t = accel(:,1);
    ay_t = accel(:,2);
    az_t = accel(:,3);
    gx_t = gyro(:,1);
    gy_t = gyro(:,2);
    gz_t = gyro(:,3);

    % 数据存入缓冲区
    ax_data = dataShiftAppend(ax_data,ax_t);
    ay_data = dataShiftAppend(ay_data,ay_t);
    az_data = dataShiftAppend(az_data,az_t);
    data_cnt = data_cnt + 1;
    if data_cnt>(buffer_length/sam_per_read)
        buffer_available = true;
    end

    %动态绘图
    addpoints(an_ax,duration_ms,ax_t);
    addpoints(an_ay,duration_ms,ay_t);
    addpoints(an_az,duration_ms,az_t);
%     addpoints(an_gx,duration_ms,gx_t);
%     addpoints(an_gy,duration_ms,gy_t);
%     addpoints(an_gz,duration_ms,gz_t);
    drawnow

end

function data_source = dataShiftAppend(data_source,data_append)
    data_source = circshift(data_source,-sam_per_read);
    data_source(1,buffer_length-sam_per_read+1:buffer_length) = data_append;
end