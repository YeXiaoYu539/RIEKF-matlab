clc
clear
close all
load("Data\traindata.mat");


% Prepare data for LSTM
numFeatures = 3;  % Number of input features (Pitch Rate, Lift Coefficient, Elevator Angle)
numResponses = 1;  % Number of output responses (Attack Angle)
% numTimeStepsTrain = floor(0.9*numel(data.Time));  % Use 90% of the data for training
X = [traindata.PitchRate, traindata.LiftCoefficient, traindata.ElevatorAngle];  % Input data
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




% Create a neural network model
net = feedforwardnet([50, 50]);
net.trainParam.epochs = 500;  % 训练轮数
% Train the network
net = train(net, X', Y');

% Predict the attack angle
y_pred_norm = net(X');

% De-normalize the predicted attack angle
y_pred = y_pred_norm' * sig_Y + mu_Y;

% Convert attack angles to degrees
AOAref = rad2deg(traindata.AttackAngle);

AOA_NN = rad2deg(y_pred_norm);


%%
Time = 0:0.01:(size(traindata.AttackAngle,1) - 1)*0.01;
% Plot actual and calculated attack angles
figure;
plot(Time, AOAref, 'DisplayName', 'Actual Attack Angle','LineWidth',2);
hold on;
plot(Time, AOA_NN, ':', 'DisplayName', 'Calculated Attack Angle (Filtered + NN + LPF 15Hz)','LineWidth',2);
xlabel('Time');
ylabel('Attack Angle (degree)');
legend;
hold off;

%%
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

% Create a neural network model
net = feedforwardnet([50, 50]);
net.trainParam.epochs = 300;  % 训练轮数
% Train the network
net = train(net, X_norm', Y_norm');

% Predict the attack angle
y_pred_norm = net(X_norm');

% De-normalize the predicted attack angle
y_pred = y_pred_norm' * sig_Y + mu_Y;
