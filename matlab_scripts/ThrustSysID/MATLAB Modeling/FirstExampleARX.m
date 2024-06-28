% Simulate input-output data
A = [1 -1.5 0.7]; % Coefficients for the autoregressive part
B = [0 0.5 0.4];  % Coefficients for the exogenous input part

N = 1000; % Number of data points
u = randn(N, 1); % Input signal
y = filter(B, A, u) + 0.1*randn(N, 1); % Output signal with some noise

% Plot the input and output signals
figure;
subplot(2, 1, 1);
plot(u);
title('Input Signal');
subplot(2, 1, 2);
plot(y);
title('Output Signal');

% Fit an ARX model to the data
data = iddata(y, u, 1); % Create iddata object

na = 2; % Order of the autoregressive part
nb = 2; % Order of the exogenous input part
nk = 0; % Input-output delay

model = arx(data, [na nb nk]); % Estimate the ARX model
disp(model); % Display the model parameters


% Validate the model
figure()
compare(data, model); % Compare the model output with the actual output
figure;
resid(data, model); % Plot the residuals
