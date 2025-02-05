folder_path = '.\trainflightrawdata';
files_list = dir(fullfile(folder_path,'*.csv'));

num_files = numel(files_list);
Alldata_laishui = cell(num_files,1); 
for i = 1:num_files         
    filename = files_list(i).name; % 获取文件名
    filepath = fullfile(files_list(i).folder, filename); % 获取文件路径
    disp(filepath)
    All = readdata(filepath);
    Alldata_laishui{i} = sim("GNSS_INS230612_AOA.slx");
    Alldata_laishui{i}.filename = filepath;
    disp(Alldata_laishui{i});
end
%%
filepath = fullfile(files_list(2).folder, filename)
All = readdata(filepath);
Alldata_laishui{2} = sim("GNSS_INS230612_AOA.slx");
Alldata_laishui{2}.filename = filepath;
disp(Alldata_laishui{2});
%% data process

% 125_飞行数据20210714_涞水.csv':                10-180s    229
%-------飞行数据20210715_不纠_495(空速补偿1):     10-140     230
%146_飞行数据20210709_涞水初赛_ID4.c            25-190     231
% 156_飞行数据20210709_涞水初赛_ID5.            25-205     232
% ------171_飞行数据20210718_飞行数据           8-135s      233
% 172_飞行数据20210709_涞水初赛_ID1             15-155      234
% 208_飞行数据20210707_4架_0号_51               15-150s     235
% 209_飞行数据20210709_涞水初赛_ID3             20-190s     236
% 210_飞行数据20210715_涞水_纠侧偏               10-150s    237
% 211_飞行数据20210709_涞水初赛_ID0             15-140s     238
% 21_飞行数据切纯惯导                           10-150s     239
% -------25_切纯惯导纠侧偏20210709 10-150s            240

for i = 1:12
figure(Name=num2str(i))
disp(Alldata_laishui{i}.filename);
plot(Alldata_laishui{i}.tout,Alldata_laishui{i}.NavData.RI_NavData.AOA_SA_Wind.AOA.Data(:,1),'LineWidth',2,'Color',color1);
hold on;
plot(Alldata_laishui{i}.tout,Alldata_laishui{i}.NavData.OtherData.AOA_direct_Deg.Data(:,1),'LineWidth',2,'Color',color2);
end
%% data production
Alldata_laishui{1}.starttime = 10;Alldata_laishui{1}.endtime = 180;
Alldata_laishui{2}.starttime = 10;Alldata_laishui{2}.endtime = 140;
Alldata_laishui{3}.starttime = 25;Alldata_laishui{3}.endtime = 190;
Alldata_laishui{4}.starttime = 25;Alldata_laishui{4}.endtime = 205;
Alldata_laishui{5}.starttime = 8;Alldata_laishui{5}.endtime = 135;
Alldata_laishui{6}.starttime = 15;Alldata_laishui{6}.endtime = 155;
Alldata_laishui{7}.starttime = 15;Alldata_laishui{7}.endtime = 150;
Alldata_laishui{8}.starttime = 20;Alldata_laishui{8}.endtime = 190;
Alldata_laishui{9}.starttime = 10;Alldata_laishui{9}.endtime = 150;
Alldata_laishui{10}.starttime = 15;Alldata_laishui{10}.endtime = 140;
Alldata_laishui{11}.starttime = 10;Alldata_laishui{11}.endtime = 150;
Alldata_laishui{12}.starttime = 10;Alldata_laishui{12}.endtime = 150;
AOAdata_laishui = cell(12,1);

ts = 0.01;
mess = 6.9;
g = 9.79;
rou = 1.225;
a = 1.4;
b = 0.36;
S = a*b;

for ii = 1:12
    timeperiod = Alldata_laishui{ii}.starttime/ts:Alldata_laishui{ii}.endtime/ts;
    AOAdata_laishui{ii}.time = Alldata_laishui{ii}.tout(timeperiod,1);
    AOAdata_laishui{ii}.V_airspeed = ...
        Alldata_laishui{ii}.NavData.SensorData.VairspeedData.Data(timeperiod,1);
    AOAdata_laishui{ii}.alpha = ...
        Alldata_laishui{ii}.NavData.RI_NavData.AOA_SA_Wind.AOA.Data(timeperiod,1)/57.3;
    AOAdata_laishui{ii}.beta = ...
        Alldata_laishui{ii}.NavData.RI_NavData.AOA_SA_Wind.SA.Data(timeperiod,1)/57.3;
    AOAdata_laishui{ii}.ax = Alldata_laishui{ii}.NavData.SensorData.accl_B_m_s2_LPF.Data(timeperiod,1);
    AOAdata_laishui{ii}.ay = Alldata_laishui{ii}.NavData.SensorData.accl_B_m_s2_LPF.Data(timeperiod,2);
    AOAdata_laishui{ii}.az = Alldata_laishui{ii}.NavData.SensorData.accl_B_m_s2_LPF.Data(timeperiod,3);

    AOAdata_laishui{ii}.wx = Alldata_laishui{ii}.NavData.SensorData.gyro_B_rad_s_LPF.Data(timeperiod,1);
    AOAdata_laishui{ii}.wy = Alldata_laishui{ii}.NavData.SensorData.gyro_B_rad_s_LPF.Data(timeperiod,2);
    AOAdata_laishui{ii}.wz = Alldata_laishui{ii}.NavData.SensorData.gyro_B_rad_s_LPF.Data(timeperiod,3);

    AOAdata_laishui{ii}.elevator = Alldata_laishui{ii}.NavData.SensorData.elevator_aileron_rudder_deg.Data(timeperiod,1)/57.3;
    AOAdata_laishui{ii}.rudder = Alldata_laishui{ii}.NavData.SensorData.elevator_aileron_rudder_deg.Data(timeperiod,3)/57.3;
    AOAdata_laishui{ii}.aileron = Alldata_laishui{ii}.NavData.SensorData.elevator_aileron_rudder_deg.Data(timeperiod,2)/57.3;

    AOAdata_laishui{ii}.q = 0.5 * rou * AOAdata_laishui{ii}.V_airspeed.^2;

    AOAdata_laishui{ii}.azs = -AOAdata_laishui{ii}.ax .* sin(AOAdata_laishui{ii}.alpha) + AOAdata_laishui{ii}.az .* cos(AOAdata_laishui{ii}.alpha);
    AOAdata_laishui{ii}.ays = -AOAdata_laishui{ii}.ax .* cos(AOAdata_laishui{ii}.alpha).*sin(AOAdata_laishui{ii}.beta) ...
         + AOAdata_laishui{ii}.ay .* cos(AOAdata_laishui{ii}.beta) ... 
         - AOAdata_laishui{ii}.az .* sin(AOAdata_laishui{ii}.alpha).* sin(AOAdata_laishui{ii}.beta);
   
    AOAdata_laishui{ii}.Cl = -mess * AOAdata_laishui{ii}.azs ./ (AOAdata_laishui{ii}.q .* S);
    AOAdata_laishui{ii}.Cy = mess * AOAdata_laishui{ii}.ays ./ (AOAdata_laishui{ii}.q .* S);
end

AOAdata_laishui{1}.starttime = 10;AOAdata_laishui{1}.endtime = 180;
AOAdata_laishui{2}.starttime = 10;AOAdata_laishui{2}.endtime = 140;
AOAdata_laishui{3}.starttime = 25;AOAdata_laishui{3}.endtime = 190;
AOAdata_laishui{4}.starttime = 25;AOAdata_laishui{4}.endtime = 205;
AOAdata_laishui{5}.starttime = 8;AOAdata_laishui{5}.endtime = 135;
AOAdata_laishui{6}.starttime = 15;AOAdata_laishui{6}.endtime = 155;
AOAdata_laishui{7}.starttime = 15;AOAdata_laishui{7}.endtime = 150;
AOAdata_laishui{8}.starttime = 20;AOAdata_laishui{8}.endtime = 190;
AOAdata_laishui{9}.starttime = 10;AOAdata_laishui{9}.endtime = 150;
AOAdata_laishui{10}.starttime = 15;AOAdata_laishui{10}.endtime = 140;
AOAdata_laishui{11}.starttime = 10;AOAdata_laishui{11}.endtime = 150;
AOAdata_laishui{12}.starttime = 10;AOAdata_laishui{12}.endtime = 150;

%remove the 2 5 12 data
removenum = [2,5,12];
AOAdata_laishui(removenum) = [];
AOAdata_laishui = AOAdata_laishui(~cellfun('isempty', AOAdata_laishui)); % 移除空元素

%% train data
% Perform linear regression
% 遍历 Cell 数组中的每个元素
traindata.SlideAngle = [];traindata.RollRate = [];traindata.YawRate = [];
traindata.RudderAngle = [];traindata.SideCoefficient = [];

for i = 1:numel(AOAdata_laishui)
    % 提取当前元素中的结构体字段值
    traindata.SlideAngle = [traindata.SlideAngle; AOAdata_laishui{i}.beta];
    traindata.RollRate = [traindata.RollRate; AOAdata_laishui{i}.wx];
    traindata.YawRate = [traindata.YawRate; AOAdata_laishui{i}.wz];
    traindata.RudderAngle = [traindata.RudderAngle; AOAdata_laishui{i}.rudder];
    traindata.SideCoefficient = [traindata.SideCoefficient; AOAdata_laishui{i}.Cy];
end
%%
%traindata = movfilter(traindata);
X_filtered = [traindata.AttackAngle, traindata.PitchRate, traindata.ElevatorAngle];
y_filtered = traindata.LiftCoefficient;
model_filtered = fitlm(X_filtered, y_filtered);

% Get model coefficients
coefficients_filtered = model_filtered.Coefficients.Estimate;

%% check
% Calculate attack angle from model
traindata.CalculatedAttackAngleFiltered = (traindata.LiftCoefficient - ...
    coefficients_filtered(1) - ...
    coefficients_filtered(3) * traindata.PitchRate - ...
    coefficients_filtered(4) * traindata.ElevatorAngle) / coefficients_filtered(2);

% Convert attack angles to degrees
traindata.AttackAngle = rad2deg(traindata.AttackAngle);
traindata.CalculatedAttackAngleFiltered = rad2deg(traindata.CalculatedAttackAngleFiltered);

figure(Name='AOA compare');
traindata.Time = 0:0.01:(size(traindata.AttackAngle,1) - 1)*0.01;
plot(traindata.Time, traindata.AttackAngle, 'DisplayName', 'Actual Attack Angle','LineWidth',2);
hold on;
% plot(data.Time, data.CalculatedAttackAngleFiltered, '--', 'DisplayName', 'Calculated Attack Angle (Filtered)','LineWidth',2);
plot(traindata.Time, traindata.CalculatedAttackAngleFiltered, ':', 'DisplayName', 'Calculated Attack Angle (Filtered + LPF 15Hz)','LineWidth',2);
xlabel('Time');
ylabel('Attack Angle (degree)');
legend;
hold off;
%% function
function All = readdata(path)
    data = xlsread(path,1);
    startNum = 3;
    endNum = length(data(:,1));
    % endNum = 3000;
    dataRange = startNum:endNum;
    %% 时间
    All.t = (startNum:endNum)*0.01;All.vt = data(dataRange,3);
    % All.t = All.t';
    %% 高度
    All.altitude = data(dataRange,4);
    %% 速度
    All.vn = data(dataRange,5);
    All.ve = data(dataRange,6);
    All.mhdot = data(dataRange,7);
    
    All.psd = data(dataRange,8);
    All.yrw = data(dataRange,9);
    All.xrw = data(dataRange,10);
    All.drange = data(dataRange,13);
    All.mh = data(dataRange,14);
    All.phiL = data(dataRange,15);
    All.thetaL = data(dataRange,16);
    All.psiL = data(dataRange,17);
    All.p = data(dataRange,18);
    All.q = data(dataRange,19);
    All.r = data(dataRange,20);
    All.Nx = data(dataRange,21);
    All.Ny = data(dataRange,22);
    All.Nz = data(dataRange,23);
    All.Qbar = data(dataRange,24);
    All.Vc = data(dataRange,25);
    All.href = data(dataRange,27);
    %% 三维位置
    All.Xnorth = data(dataRange,30);
    All.Yeast = data(dataRange,31);
    All.hrefAL = data(dataRange,32);
    All.apthL = data(dataRange,37);
    All.aplonL = data(dataRange,38);
    All.aplatL = data(dataRange,39);
    All.apyawL = data(dataRange,40);
    All.apnwsL = data(dataRange,41);
    All.apbrkL = data(dataRange,42);
    All.thrustcmd = data(dataRange,43);
    All.vtcmd = data(dataRange,44);
    All.nzcmd = data(dataRange,45);
    All.nycmd = data(dataRange,46);
    All.phicmd = data(dataRange,47);
    All.gamacmd = data(dataRange,48);
    All.hcmd = data(dataRange,49);
    All.psdcmd = data(dataRange,50);
    All.yrwcmd = data(dataRange,51);
    All.k = data(dataRange,54);
    All.auto = data(dataRange,61);
    All.RClost = data(dataRange,68);
    All.LockCH = data(dataRange,69);
    All.TakeoffCH = data(dataRange,70);
    All.ModeselectCH = data(dataRange,71);
    All.ManualCH = data(dataRange,72);
    All.LockUL = data(dataRange,73);
    All.TakeoffUL = data(dataRange,74);
    All.ModeSelect = data(dataRange,75);
    All.Manual = data(dataRange,76);
    All.thrust = data(dataRange,77);
    All.decl = data(dataRange,78);
    All.dacl = data(dataRange,79);
    All.drcl = data(dataRange,80);
    All.nwscl = data(dataRange,81);
    All.appc = data(dataRange,82);
    All.aprc = data(dataRange,83);
    All.apyc = data(dataRange,84);
    All.fmphase = data(dataRange,86);
    All.door_incamx = data(dataRange,87);
    All.door_incamy = data(dataRange,88);
    All.door_incamz = data(dataRange,89);
    All.can_indoorx = data(dataRange,93);
    All.can_indoory = data(dataRange,94);
    All.can_indoorz = data(dataRange,95);
    %% 经纬度
    All.lon = data(dataRange,103);
    All.lat = data(dataRange,104);
    
    All.D_confidenceFG  = data(dataRange,105);
    All.CVmode = data(dataRange,107);
    All.INS_phi = data(dataRange,108);
    All.INS_theta = data(dataRange,109);
    All.INS_psi = data(dataRange,110);
    All.INS_vn = data(dataRange,111);
    All.INS_ve = data(dataRange,112);
    All.INS_vd = data(dataRange,113);
    All.INS_xn = data(dataRange,114);
    All.INS_xe = data(dataRange,115);
    All.INS_xd = data(dataRange,116);
    All.GPSMode = data(dataRange,117);
    All.Alt_Ps = data(dataRange,118);
    All.baro = data(dataRange,120);
    All.mx = data(dataRange,124);
    All.my = data(dataRange,125);
    All.mz = data(dataRange,126);
end

function data = movfilter(data)
window_size = 10;
    data.PitchRateFiltered = movmean(data.PitchRate, window_size, 'omitnan');
    
    % Drop the rows that contain NaN values due to the rolling window
    data(isnan(data.PitchRateFiltered), :) = [];
end