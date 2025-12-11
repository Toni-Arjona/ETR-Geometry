close all
clc
clear all

tiempos = zeros(1, 70);
idx = 1;

for gear_ratio = 8:0.1:15
    tiempo = simu_acc(gear_ratio);
    tiempos(idx) = tiempo;

    idx = idx +1;

end

plot(8:0.1:15, tiempos)