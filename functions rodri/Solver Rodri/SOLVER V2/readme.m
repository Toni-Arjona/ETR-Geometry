%% ====================================================================================================================
%  MÉTODO ANALÍTICO DE RODRIGUES - Solución cerrada de restricciones
%  ====================================================================================================================
%
%  Todas las ecuaciones a resolver tienen la forma:
%
%    norm(offset + rodrigues_rot(v, k, th) - target) = L
%
%  donde:
%    offset         - Punto de referencia del eje de rotación
%    rodrigues_rot  - Componente de rotación pura
%    target         - Punto del chasis respecto al que se cumple la restricción
%    L              - Longitud del brazo (distancia a ese target)
%
%  La fórmula de Rodrigues se expresa como la descomposición en componentes axial y radial sobre el eje:
%
%    rodrigues_rot(v, k, th) = a + r*cos(th) + s*sin(th)
%
%  donde:
%    a = dot(k,v)*k    Componente axial. Determina a qué altura a lo largo del eje está el centro de la
%                       circunferencia, con el "punto offset" como referencia. Se determina  como la
%                       proyección del vector v sobre el eje k, con el módulo de ese vector como dot(k,v) y
%                       dirección k.
%
%    r = v - a          Componente radial. Su módulo es el radio de la circunferencia, la distancia ortogonal del
%                       punto a rotar al eje k. Se calcula como v - a.
%
%    s = cross(k,v)     Componente tangencial. Determina el plano de la rotación junto con r. Se calcula
%                       como cross(k,v).
%
%  Sustituyendo y elevando al cuadrado ambos lados:
%
%    || offset + a + r*cos(th) + s*sin(th) - target ||^2 = L
%
%  Agrupamos todo lo que no depende de th como q = offset + a - target:
%
%    || q + r*cos(th) + s*sin(th) ||^2 = L
%
%  Expandiendo el cuadrado y aplicando que |r| = |s| y r ⊥ s:
%
%    q·r·cos(th) + q·s·sin(th) = (L^2 - |r|^2 - |q|^2) / 2
%
%  Definiendo:
%    B = dot(q, r)                          C = dot(q, s)                          D = (L^2 - |r|^2 - |q|^2) / 2
%
%  Queda:  B*cos(th) + C*sin(th) = D
%
%  Cuya solución cerrada es:
%
%    th = atan2(C, B) ± acos(D / sqrt(B^2 + C^2))
%
%  Esto da dos soluciones, pero es tan superior a nivel de eficiencia que aún calculándolo dos veces y escogiendo
%  la solución más cercana a la configuración inicial es unas 20 veces más rápido que el método numérico con
%  semilla caliente.
% 
%  Este el fundamento matemático, dado esto, el proceso para resolver el sistema es:

%  1. Resolver el ángulo del rocker que cumple la distancia de damper introducida según la compresión del damper.
%       Incógnita - ángulo rocker;      Restricción - extensión total damper

%  2. Resolver el ángulo del rocker que cumple la distancia de damper introducida según la compresión del damper.
%       Incógnita - ángulo ARB;      Restricción - longitud ARB link.

%  3. Resolver el ángulo del upper wishbone que cumple con la longitud del push
%       Incógnita - ángulo upper wishbone;      Restricción - longitud push

%  4. Resolver el ángulo del lower wishbone que cumple con la longitud del kingpin
%       Incógnita - ángulo lower wishbone;      Restricción - distancia entre knuckles

%  5. Resolver el ángulo toe de la mangueta que cumple con la longitud del tie rod
%       Incógnita - toe mangueta;      Restricción - longitud tie rod

%  Antes de comenzar con los cálculos, se guardan en la memoria las longitudes, ejes unitarios y cualquier otra constante
%  geométrica del sistema.

%  También se guarda la solución anterior de cada rueda, con un wheel_ID, para tener un criterio para escoger entre ambas 
%  soluciones de la ecuación obtenida en la explicación anterior
%  ====================================================================================================================