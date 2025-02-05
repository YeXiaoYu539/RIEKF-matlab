clc
clear
close all
%compare different proscess methods
%% load data
load("Data\traindata.mat");
load('Data\AlllaishuiAOA0815.mat');
%%
%traindata = movfilter(traindata);
X_filtered = [traindata.AttackAngle, traindata.PitchRate, traindata.ElevatorAngle];
y_filtered = traindata.LiftCoefficient;
model_filtered = fitlm(X_filtered, y_filtered);

% Get model coefficients
Lift_coefficients_filtered = model_filtered.Coefficients.Estimate;

%%
i = 3;

alpha_ref = AOAdata_laishui{i}.alpha;
wy = AOAdata_laishui{i}.wy;
elevator = AOAdata_laishui{i}.elevator;
Cl = AOAdata_laishui{i}.Cl;

CalculatedAttackAngleLinear = (Cl - ...
    Lift_coefficients_filtered(1) - ...
    Lift_coefficients_filtered(3) * wy - ...
    Lift_coefficients_filtered(4) * elevator) / Lift_coefficients_filtered(2) * 57.3;

%% LSTM 200U predict
load("NNModel\lstmnet.mat");
sig_X = [0.1167;0.1277;0.0579];
sig_Y = 0.0451;
mu_X = [0.0587;0.5185;-0.0515];
mu_Y = 0.1011;

X = [wy, Cl, elevator];  % Input data

X_norm = (X - mu_X') ./ sig_X';
% Prepare the data for LSTM
% XTrain = X(1:numTimeStepsTrain+1,:);
% YTrain = Y(1:numTimeStepsTrain+1);

% Standardize the data
YPred_1 = predict(net,X_norm');

CalculatedAttackAngleLSTM = (YPred_1 * sig_Y + mu_Y)*57.3;

%% LSTM 100U predict
load("NNModel\LSTM_100U_AOA.mat");
sig_X = [0.1167;0.1277;0.0579];
sig_Y = 0.0451;
mu_X = [0.0587;0.5185;-0.0515];
mu_Y = 0.1011;

X = [wy, Cl, elevator];  % Input data

X_norm = (X - mu_X') ./ sig_X';
% Prepare the data for LSTM
% XTrain = X(1:numTimeStepsTrain+1,:);
% YTrain = Y(1:numTimeStepsTrain+1);

% Standardize the data
YPred_100U = predict(net,X_norm');

CalculatedAttackAngleLSTM = (YPred_1 * sig_Y + mu_Y)*57.3;

%% NN 
load('NNModel\AOANN.mat');

% Predict the attack angle
y_pred_norm = net(X');
% De-normalize the predicted attack angle
% y_pred = y_pred_norm' * sig_Y + mu_Y;
% Convert attack angles to degrees
AOA_NN = rad2deg(y_pred_norm);

%% NN 200Units
load('NNModel\AOABP_100U_0226.mat');

% Predict the attack angle
y_pred_norm_0226 = net(X');
% De-normalize the predicted attack angle
% y_pred = y_pred_norm' * sig_Y + mu_Y;
% Convert attack angles to degrees
AOA_NN_0226 = rad2deg(y_pred_norm_0226);
%% load SA data
load("Data\traindataSA0815.mat");

X = [traindata.SlideAngle, traindata.RudderAngle, traindata.RollRate, traindata.YawRate];  % Input data
Y = traindata.SideCoefficient;  % Output data
model_filtered_SA = fitlm(X, Y);

% Get model coefficients
Side_coefficients = model_filtered_SA.Coefficients.Estimate;

%%
i = 3;

beta_ref = AOAdata_laishui{i}.beta;
wx = AOAdata_laishui{i}.wx;
wz = AOAdata_laishui{i}.wz;

rudder = AOAdata_laishui{i}.rudder;
Cy = AOAdata_laishui{i}.Cy;

CalculatedSALinear = (Cy - ...
    Side_coefficients(1) - ...
    Side_coefficients(3) * rudder - ...
    Side_coefficients(4) * wx - ...
    Side_coefficients(5) * wz)/ Side_coefficients(2) * 57.3;


%%
load("NNModel\lstmnetSA0826.mat");
sig_X = [0.2380; 0.1523 ;0.0429;0.1379];
sig_Y = 0.0690;
mu_X = [0.0068;-0.0055;-0.0070;-0.0450];
mu_Y = -0.0228;


X = [wx, Cy, rudder, wz];  % Input data


X_norm = (X - mu_X') ./ sig_X';
% Prepare the data for LSTM
% XTrain = X(1:numTimeStepsTrain+1,:);
% YTrain = Y(1:numTimeStepsTrain+1);

% Standardize the data
YPred_1 = predict(net,X_norm');

CalculatedSALSTM = (YPred_1 * sig_Y + mu_Y)*57.3;
%% 
load('NNModel\SANN0827.mat');
% Predict the attack angle
y_pred_norm = net(X_norm');
% De-normalize the predicted attack angle
y_pred = y_pred_norm' * sig_Y + mu_Y;
% Convert attack angles to degrees
SA_NN = rad2deg(y_pred);

%% 
load('NNModel\SABP_100U_0226.mat');
% Predict the attack angle
y_pred_norm_0226 = net(X_norm');
% De-normalize the predicted attack angle
y_pred_0226 = y_pred_norm_0226' * sig_Y + mu_Y;
% Convert attack angles to degrees
SA_NN_0226 = rad2deg(y_pred_0226);
%% plot
color4 = [252 160 93]/255;
color4 = [205 20 20]/255;
color5 = [190 190 190]/255;
color3 = [84 134 135]/255;
color1 = [71 51 53]/255;
color2 = [189 30 30]/255;
color6 = [252 160 93]/255;
% 6 3 5 4
figure(Name='AOA compare');
Time = 0:0.01:(size(alpha_ref,1) - 1)*0.01;
subplot1 = subplot(2,1,1)
hold on;
plot(Time, CalculatedAttackAngleLinear,'LineWidth',1.5,'Color',[color5, 0.5]);
plot(Time, AOA_NN,'LineWidth',2,'LineStyle','-','Color',[color3, 0.5]);
plot(Time, AOA_NN_0226,'LineWidth',2,'LineStyle','-','Color',[color4, 0.5]);

plot(Time, CalculatedAttackAngleLSTM,'LineWidth',2,'Color',color4);
plot(Time, alpha_ref*57.3,'LineWidth',2,'LineStyle','-','Color',color6);

ylabel('$$\alpha\,$$[deg]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'Least Squares','BP','LSTM','Ref'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);
axis([0 180 -3 23])


subplot1 = subplot(2,1,2)
hold on;
plot(Time, CalculatedSALinear/57.3,'LineWidth',1.5,'Color',[color5, 0.5]);
plot(Time, SA_NN,'LineWidth',2,'Color',[color3,0.5]);
plot(Time, CalculatedSALSTM,'LineWidth',2,'Color',color4);
plot(Time, beta_ref*57.3,'LineWidth',2,'LineStyle','-','Color',color6);
ylabel('$$\beta\,$$[deg]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'Least Squares','BP','LSTM','Ref'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);
axis([0 180 -8 13])


%%
figure(Name='AOA compare');
Time = 0:0.01:(size(alpha_ref,1) - 1)*0.01;
subplot1 = subplot(2,1,1)
hold on;
%plot(Time, CalculatedAttackAngleLinear,'LineWidth',1.5,'Color',[color5, 0.5]);
plot(Time, AOA_NN,'LineWidth',2,'LineStyle','-','Color',[color3, 0.7]);
plot(Time, AOA_NN_0226,'LineWidth',2,'LineStyle','--','Color',[color5, 0.7]);

plot(Time, CalculatedAttackAngleLSTM,'LineWidth',2,'Color',color4);
plot(Time, alpha_ref*57.3,'LineWidth',2,'LineStyle','-','Color',color6);

ylabel('$$\alpha\,$$[deg]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'BP 100U','BP 200U','LSTM','Ref'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);
axis([0 180 -3 23])


subplot1 = subplot(2,1,2)
hold on;
plot(Time, SA_NN,'LineWidth',2,'Color',[color3,0.7]);
plot(Time, SA_NN_0226,'LineWidth',2,'LineStyle','--','Color',[color5,0.7]);

plot(Time, CalculatedSALSTM,'LineWidth',2,'Color',color4);
plot(Time, beta_ref*57.3,'LineWidth',2,'LineStyle','-','Color',color6);
ylabel('$$\beta\,$$[deg]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'BP 100U','BP 200U','LSTM','Ref'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);
axis([0 180 -8 13])

%%
LS_AOA_std = mean(abs(CalculatedAttackAngleLinear - alpha_ref*57.3))
LS_AOA_rmse = rmse(CalculatedAttackAngleLinear,alpha_ref*57.3)
NN_AOA_std = mean(abs(AOA_NN' - alpha_ref*57.3))
NN_AOA_rmse = rmse(AOA_NN',alpha_ref*57.3)
LSTM_AOA_std = mean(abs(CalculatedAttackAngleLSTM' - alpha_ref*57.3))
LSTM_AOA_rmse = rmse(CalculatedAttackAngleLSTM',alpha_ref*57.3)

LS_SA_std = mean(abs(CalculatedSALinear/57.3 - beta_ref*57.3))
LS_SA_rmse = rmse(CalculatedSALinear/57.3,beta_ref*57.3)
NN_SA_std = mean(abs(SA_NN - beta_ref*57.3))
NN_SA_rmse = rmse(SA_NN,beta_ref*57.3)
LSTM_SA_std = mean(abs(CalculatedSALSTM' - beta_ref*57.3))
LSTM_SA_rmse = rmse(CalculatedSALSTM',beta_ref*57.3)


%%
