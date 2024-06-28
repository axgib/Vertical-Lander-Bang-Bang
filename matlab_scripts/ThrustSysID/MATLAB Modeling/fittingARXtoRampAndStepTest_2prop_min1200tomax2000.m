% Load CSV file into a table
ramp_filename = 'RampTest_avgSamp1_min1200_mxVl2000_doubProp_3_fullBatt_2024-06-27_135603.csv';
step_filename = 'myStepTest_onestep_avgSamp1_mxVl2000_doubProp_1_fullBatt_2024-06-27_135756.csv';
ramp_data = readtable(ramp_filename);
step_data = readtable(step_filename);

% Display the first few rows of the table
disp(head(ramp_data));
%%

ramp_u = ramp_data.ESCSignal__s_;
ramp_y = ramp_data.Thrust_kgf_;
ramp_t = ramp_data.Time_s_;

step_u = step_data.ESCSignal__s_;
step_y = step_data.Thrust_kgf_;
step_t = step_data.Time_s_;

%% Plot our Experimental Data
subplot(2,1,1)
plot(ramp_t, ramp_u)
hold on
plot(step_t, step_u)
xlabel("Time(s)")
ylabel("ESC(micoSec)")
legend(["ramp", "step"])

subplot(2,1,2)
plot(ramp_t, ramp_y)
hold on;
plot(step_t, step_y)
xlabel("Time(s)")
ylabel("Thrust(kgF)")
legend(["ramp", "step"])

%% Build ramp model
na = 10;
nb = 10;
nk = 0;

rampData = iddata(ramp_y, ramp_u, 1);
ramp_model = arx(rampData, [na, nb, nk]);

disp(ramp_model)

%% Build step model
na = 10;
nb = 10;
nk = 0;

stepData = iddata(step_y, step_u, 1);
step_model = arx(stepData, [na, nb, nk]);

disp(step_model)
%% Compare the model to the data

figure()
subplot(3,1,1)
compare(rampData, ramp_model)
title("Simulated Response (rampData, rampModel)")

subplot(3,1,2)
compare(stepData, step_model)
title("Simulated Response (stepData, stepModel)")

subplot(3,1,3)
compare(stepData, ramp_model)
title("Simulated Response (stepData, rampModel)")

%%
ts = 0.02 %s
rampG = tf(ramp_model.B, ramp_model.A, ts)
stepG = tf(step_model.B, step_model.A, ts)
%%

oe_rampModel = oe(rampData, [5, 5, 0])
oe_stepModel = oe(stepData, [5, 5, 0])

figure()
subplot(3,1,1)
compare(rampData, oe_rampModel)
title("Simulated Response (rampData, oe-rampModel)")

subplot(3,1,2)
compare(stepData, oe_stepModel)
title("Simulated Response (stepData, oe-stepModel)")

subplot(3,1,3)
compare(stepData, oe_rampModel)
title("Simulated Response (stepData, oe-rampModel) - cross validation")
