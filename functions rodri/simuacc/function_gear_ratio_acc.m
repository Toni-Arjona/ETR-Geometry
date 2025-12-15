close all
clc
clear all

tiempos = zeros(1, 50);
idx = 1;

for gear_ratio = 7:0.1:12
    tiempo = simu_acc_function(gear_ratio)
    tiempos(idx) = tiempo;

    idx = idx +1;

end

plot(7:0.1:12, tiempos)
