clc; clear;

% Rango de ratios a probar (ej: de 8 a 16 con saltos de 0.5)
ratios = 10:0.01:14; 
tiempos = zeros(size(ratios));

% Bucle de optimización
for i = 1:length(ratios)
    tiempos(i) = simulap_func(ratios(i));
end

% Resultados
[mejor_tiempo, idx] = min(tiempos);
mejor_ratio = ratios(idx);

fprintf('Mejor tiempo: %.3f s | Mejor Ratio: %.2f\n', mejor_tiempo, mejor_ratio);

plot(ratios, tiempos, '-o');
xlabel('Gear Ratio'); ylabel('Tiempo de vuelta [s]');
grid on;