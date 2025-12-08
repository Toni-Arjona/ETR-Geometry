clc; clear;

% Rango de ratios a probar (ej: de 8 a 16 con saltos de 0.5)
gear_ratio = 10.5:0.05:12; 
tiempos = zeros(size(gear_ratio));

% Bucle de optimización
for i = 1:length(gear_ratio)
    tiempos(i) = simulap_func(gear_ratio(i));
end

% Resultados
[mejor_tiempo, idx] = min(tiempos);
mejor_ratio = gear_ratio(idx);

fprintf('Mejor tiempo: %.3f s | Mejor Ratio: %.2f\n', mejor_tiempo, mejor_ratio);

plot(gear_ratio, tiempos, '-o');
xlabel('Gear Ratio'); ylabel('Tiempo de vuelta [s]');
grid on;