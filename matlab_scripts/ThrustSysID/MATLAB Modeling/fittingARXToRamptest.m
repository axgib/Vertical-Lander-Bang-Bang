% Load CSV file into a table
filename = 'RampTest_avgSamp1_mxVl2000_doubProp_1_2024-06-27_132115.csv';  % Replace with your CSV file name
data = readtable(filename);

% Display the first few rows of the table
disp(head(data));
%%

u = data.ESCSignal__s_
y = data.Thrust_kgf_
t = data.Time_s_

%%
plot(t, y)
xlabel("Time(s)")
ylabel("Thrust(kgF)")

figure()
plot(t, u)
%% Build the model
na = 2;
nb = 2;
nk = 0;

myData = iddata(y, u, 1);
model = arx(myData, [na, nb, nk]);

disp(model)

%% Compare the model to the data

figure()
compare(myData, model)

figure()
resid(myData, model)
