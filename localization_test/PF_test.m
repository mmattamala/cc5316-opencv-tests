close all;

%% Vehiculo y mapa
% Covarianza de la odometría (para simular trayectoria)
V = diag([0.1, 1*pi/180].^2);

% Creamos un vehículo
veh = Vehicle(V);

% Añadimos una trayectoria a seguir por el vehículo
veh.add_driver( RandomPath(10) );

% Creamos el mapa de [-10 a +10], con 20 landmarks (aleatorios)
map = Map(20, 10);

%% Modelo del Sensor
% Covarianza del sensor
% primer termino: Varianza puntual (distancia)
% segundo termino: Varianza rotacional
sensor_var_dist = 1;
sensor_var_rot = pi/180;
W_est = diag([sensor_var_dist, sensor_var_rot].^2);

% Sensor (de rango)
% Recibe el vehiculo asociado, el mapa, la covarianza de medicion
sensor = RangeBearingSensor(veh, map, W_est, 'animate');

%% Modelo de movimiento
% Covarianza del modelo de movimiento de las particulas
% primer termino: varianza de traslación en x
% primer termino: varianza de traslación en y
% tercer termino: varianza de cambio de orientacion
Q = diag([0.1, 0.1, 1*pi/180]).^2;
%Q = diag([1, 1, 5*pi/180]).^2;

% Función de verosimilitud que mapea error entre lo observado y esperado a
% un peso (el toolbox usa una gaussiana, asi que se requiere una covarianza)
L = diag([0.1 0.1]);

%% Filtro de Particulas

% Now we create an instance of the particle filter class
pf = ParticleFilter(veh, sensor, Q, L, 1000);

pf.run(1000);

% Grafico del mapa
clf; map.plot()
% y trayectoria
veh.plot_xy('b');
% y trayectoria estimada por EKF
pf.plot_xy('r');

%% Incerteza de la estimación (dispersión de partículas)
figure; plot(pf.std); xlabel('time step'); ylabel('standard deviation'); legend('x', 'y', '\theta'); grid       
