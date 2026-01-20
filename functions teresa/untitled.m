close all
clc
clear all


slip_int = deg2rad(-15:0.1:15);
fz = 137;
idx = 1;
camber = deg2rad(-2);
fy_results = zeros(1, length(slip_int));

for slip = slip_int
    [Fy, Mz] = tire_model_function(slip, camber, fz)
    fy_results(idx) = Fy

    idx = idx +1
end

max(fy_results)

[max_fy, idx_1] = max(fy_results);


rad2deg(slip_int(idx_1))

plot(rad2deg(slip_int), fy_results)




