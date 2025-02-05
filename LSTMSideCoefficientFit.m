% Load data
clc
clear
close all
load("Data\traindataSA0815.mat");


% Prepare data for LSTM
numFeatures = 4;  % Number of input features (Pitch Rate, Lift Coefficient, Elevator Angle)
numResponses = 1;  % Number of output responses (Attack Angle)
% numTimeStepsTrain = floor(0.9*numel(data.Time));  % Use 90% of the data for training
X = [traindata.RollRate, traindata.SideCoefficient, traindata.RudderAngle, traindata.YawRate];  % Input data
Y = traindata.SlideAngle;  % Output data

% Prepare the data for LSTM
% XTrain = X(1:numTimeStepsTrain+1,:);
% YTrain = Y(1:numTimeStepsTrain+1);

% Standardize the data
mu_X = mean(X);
sig_X = std(X);
mu_Y = mean(Y);
sig_Y = std(Y);
X_norm = (X - mu_X) ./ sig_X;
Y_norm = (Y - mu_Y) ./ sig_Y;

% Define LSTM network
inputSize = numFeatures;
outputSize = 200;
outputMode = 'sequence';
layers = [ ...
    sequenceInputLayer(inputSize)
    lstmLayer(outputSize,'OutputMode','sequence')
    fullyConnectedLayer(numResponses)
    regressionLayer];

    %'SequenceLength',outputSize,...

options = trainingOptions('adam', ...
    'ExecutionEnvironment','gpu',...
    'MaxEpochs',500, ...
    'MiniBatchSize',outputSize, ...
    'InitialLearnRate',0.005, ...
    'GradientThreshold',1, ...
    'Shuffle','never', ...
    'Verbose',0, ...
    'Plots','training-progress');

% Train LSTM network
net = trainNetwork(X_norm',Y_norm',layers,options);

% Use trained LSTM network to make predictions
% XTest = X(numTimeStepsTrain+1:end,:);
% YTest = Y(numTimeStepsTrain+1:end);
% 
% XTest = (XTest - mu) ./ sig;

[net,YPred] = predictAndUpdateState(net,X_norm');
YPred = YPred * sig_Y + mu_Y;
%%
for i = 1:size(X_norm(10000,:),1)
    %net = load("NNModel\lstmnetSA.mat");
    [~,YPred_test(i)] = predictAndUpdateState(net,X_norm(i,:)');
    YPred_test(i) = YPred_test(i)* sig_Y + mu_Y;
end
% [net,YPred] = predictAndUpdateState(net,YTrain(end));

% numTimeStepsTest = numel(XTest);
% for i = 2:numTimeStepsTest
%     [net,YPred(:,i)] = predictAndUpdateState(net,YPred(:,i-1),'ExecutionEnvironment','cpu');
% end

% Compare the predicted and actual attack angles
traindata.Time = 0:0.01:(size(traindata.SlideAngle,1) - 1)*0.01;
figure
plot(traindata.Time,traindata.SlideAngle*57.3,'LineWidth',2);
hold on;
plot(traindata.Time,YPred*57.3)
% plot(traindata.Time,YPred_test*57.3,'LineStyle','--')


%% test one dataset
ts = 0.01;
mess = 6.9;
g = 9.79;
rou = 1.225;
a = 1.4;
b = 0.36;
S = a*b;
dataset = Alldata_laishui{4};
dataset.starttime = 25;
dataset.endtime = 205;
timeperiod = dataset.starttime/ts:dataset.endtime/ts;
PitchRate = dataset.NavData.SensorData.gyro_B_rad_s_LPF.Data(timeperiod,2);

    V_airspeed = ...
        Alldata_laishui{4}.NavData.SensorData.VairspeedData.Data(timeperiod,1);
    alpha = ...
        Alldata_laishui{4}.NavData.RI_NavData.AOA_SA_Wind.AOA.Data(timeperiod,1)/57.3;
    ax = Alldata_laishui{4}.NavData.SensorData.accl_B_m_s2_LPF.Data(timeperiod,1);
    az = Alldata_laishui{4}.NavData.SensorData.accl_B_m_s2_LPF.Data(timeperiod,3);
    elevator = Alldata_laishui{4}.NavData.SensorData.elevator_aileron_rudder_deg.Data(timeperiod,1)/57.3;

    q = 0.5 * rou * V_airspeed.^2;

    azs = -ax .* sin(alpha) + az .* cos(alpha);
    Cl = -mess * azs ./ (q .* S);


LiftCoefficient = Cl;
ElevatorAngle = elevator;
X = [PitchRate, LiftCoefficient, ElevatorAngle];  % Input data
Y = traindata.AttackAngle;  % Output data

% Prepare the data for LSTM
% XTrain = X(1:numTimeStepsTrain+1,:);
% YTrain = Y(1:numTimeStepsTrain+1);

% Standardize the data
mu_X = mean(X);
sig_X = std(X);
mu_Y = mean(Y);
sig_Y = std(Y);
X_norm = (X - mu_X) ./ sig_X;
Y_norm = (Y - mu_Y) ./ sig_Y;

[~,YPred_1] = predictAndUpdateState(net,X_norm(1:10000,:)',"SequenceLength","shortest");
YPred_1 = YPred_1 * sig_Y + mu_Y;


YPred_1_Time = 0:0.01:(10000 - 1)*0.01;

% for i = 1:(size(X_norm,1) - 1)/1000
%     %net = load("NNModel\lstmnet.mat");
%     YPred_test((1000*i - 999):(1000*i +1)) = predict(net,X_norm((1000*i - 999):(1000*i +1),:)');
%     YPred_test((1000*i - 999):(1000*i +1)) = YPred_test((1000*i - 999):(1000*i +1))* sig_Y + mu_Y;
% end
%%
X_nYPredorm_store = zeros(1000,4);
YPred_test = zeros(10000,1);
for i = 1:size(X_norm(1:10000,:),1)
    X_nYPredorm_store = [X_nYPredorm_store(2:end,:);X_norm(i,:)];
    YPred_seq = predict(net,X_nYPredorm_store',"Acceleration","auto","ExecutionEnvironment","gpu");
    YPred_test(i) = YPred_seq(end);
    YPred_test(i) = YPred_test(i)* sig_Y + mu_Y;
end

YPred_seq_test = predict(net,X_norm(1:1000,:)');
YPred_seq_test_time = 0:0.01:(size(YPred_seq_test,2) - 1)*0.01;
figure
plot(traindata.Time,traindata.SlideAngle*57.3,'LineWidth',2);
hold on;
plot(YPred_1_Time,YPred_test*57.3)
% plot(YPred_1_Time,YPred_1*57.3,'LineStyle','--')
% plot(YPred_seq_test_time,YPred_seq_test,'LineWidth',1.5)
% legend('traindata','YPred_test','YPred_1','YPred_seq_test')
