clc
clear
close all
load("Data\traindata.mat");

X_filtered = [traindata.AttackAngle, traindata.PitchRate, traindata.ElevatorAngle];
% X_filtered = [traindata.AttackAngle, traindata.ElevatorAngle];

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

function data = movfilter(data)
window_size = 10;
    data.PitchRateFiltered = movmean(data.PitchRate, window_size, 'omitnan');
    
    % Drop the rows that contain NaN values due to the rolling window
    data(isnan(data.PitchRateFiltered), :) = [];
end