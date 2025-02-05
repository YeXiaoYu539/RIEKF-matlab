modelName = 'GNSS_INS240226time';
load_system(modelName);

startTime = 0; % 模拟开始时间
endTime = 220; % 模拟结束时间
set_param(modelName, 'StartTime', num2str(startTime), 'StopTime', num2str(endTime));

tic; % 开始计时
sim(modelName); % 运行Simulink模型
elapsedTime = toc; % 结束计时并获取经过的时间


modelName = 'GNSS_INS240226timeNoLSTM';
load_system(modelName);

startTime = 0; % 模拟开始时间
endTime = 220; % 模拟结束时间
set_param(modelName, 'StartTime', num2str(startTime), 'StopTime', num2str(endTime));

tic; % 开始计时
sim(modelName); % 运行Simulink模型
elapsedTime_AOAdirect = toc; % 结束计时并获取经过的时间
