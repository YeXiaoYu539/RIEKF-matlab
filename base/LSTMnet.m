function YPred_test = LSTMnet(X)
    YPred_test = 0;
    sig_X = [0.1167;0.1277;0.0579];
    sig_Y = 0.0451;
    mu_X = [0.0587;0.5185;-0.0515];
    mu_Y = 0.1011;
    X_norm = (X - mu_X) ./ sig_X;
    % size(X_norm,1)
    net = load("NNModel\lstmnet.mat");
    persistent X_nYPredorm_store;
    % YPred_seq = zeros(1000,1);
    if isempty(X_nYPredorm_store)
        X_nYPredorm_store = zeros(1000,3);
    end

    X_nYPredorm_store = [X_nYPredorm_store(2:end,:);X_norm'];
    size(X_nYPredorm_store,1);
    %[~,YPred_seq] = predictAndUpdateState(net.net,X_nYPredorm_store);
    YPred_seq = predict(net.net,X_nYPredorm_store');
    YPred_test = YPred_seq(end);
    YPred_test = double(YPred_test* sig_Y + mu_Y)*57.3;
    % YPred = 0;
    % if YPred > 30
    %     YPred = 30;
    % elseif YPred < -30
    %     YPred = -30;
    % end
end