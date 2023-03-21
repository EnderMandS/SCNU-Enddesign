% 将串口读取到的数据存入文件
clear;
readtime = 1*60; % 秒
readtime_step = 10;

com = serialport("COM8",115200,"Parity","even","Timeout",1);

% accel = zeros(300000,3);
% gyro = zeros(300000,3);
% time = zeros(300000,1);

accel = [];
gyro = [];
time = [];

flush(com);
time_start = datetime('now');
while seconds(datetime('now')-time_start)<readtime
    if com.NumBytesAvailable==0
        disp("Waiting IMU data");
        pause(readtime_step);
    end
    read_data = read(com,com.NumBytesAvailable,"uint8");
    frame_end_index = strfind(read_data,'END');
    len_frame_end_index = length(frame_end_index);
    if len_frame_end_index>1
        for i=1:len_frame_end_index
            if i==len_frame_end_index
                break
            end
            frame = read_data((frame_end_index(i)+3):(frame_end_index(i+1)-1));
            [a, g, t, tmp] = frameProcess(frame);
            if t~=0
                accel = [accel; a];
                gyro = [gyro; g];
                time = [time; t];
            end
        end
    end
end
clear com

filename = strcat('data/',string(datetime('now','Format','uuuu-MM-dd-HH-mm-ss')),'.mat');
fprintf('Saving file as %s\n', filename);
save(filename,"accel","gyro","time","tmp");
clear;
disp("Complete");

function [accel,gyro,time,tmp] = frameProcess(frame)
    % mpu_data = mpu_obj.get_raw_values() #14
    % t = time.ticks_ms().to_bytes(4, 'little') #4
    % s = sum(mpu_data + t).to_bytes(1, 'little') #1
    % data = mpu_data + t + s + b'END' # 14+4+1+3=22
    if rem(sum(frame(1:18)),256)==frame(19)
        time = frame(15)+frame(16)*2^8+frame(17)*2^16+(frame(18)&0b01111111)*2^24;
        time = time/1000; % ms to s
        accel = zeros(1,3,'double');
        gyro = zeros(1,3,'double');
        accel(1,1) = byte2double(frame(1),frame(2))/8192*9.7883; % +-2g
        accel(1,2) = byte2double(frame(3),frame(4))/8192*9.7883;
        accel(1,3) = byte2double(frame(5),frame(6))/8192*9.7883;
        gyro(1,1) = deg2rad(byte2double(frame(9),frame(10))/16.4); % +-2000°/s
        gyro(1,2) = deg2rad(byte2double(frame(11),frame(12))/16.4);
        gyro(1,3) = deg2rad(byte2double(frame(13),frame(14))/16.4);
        tmp = byte2double(frame(7),frame(8)) /340+36.53;
    else
        time=0;
    end
end

function result = byte2double(first,second)
    arguments
        first (1,1) uint8
        second (1,1) uint8
    end
    result = double(typecast(uint8([second,first]),'int16'));
end