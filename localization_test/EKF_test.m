close all;

%% Vehiculo y mapa
% Covarianza de la odometría (para simular trayectoria)
V_real = diag([0.1, 1*pi/180].^2);

% Creamos un vehículo
veh = Vehicle(V_real);

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
% Covarianza del proceso (modelo de movimiento)
% primer termino: varianza estimada de traslación por odometría
% segundo termino: varianza estimada de orientación por odometria
V_est = diag([0.1, 1*pi/180]).^2;

% Covarianza inicial del vehiculo
% primer termino: varianza x
% segundo termino: varianza y
% tercer termino: varianza orientacion
P0 = diag([0.005, 0.005, 0.001].^2);

%% Extended Kalman Filter
% Instancia de Extended Kalman Filter
ekf = EKF(veh, V_est, P0, sensor, W_est, map);

% Ejecución
ekf.run(1000);

% Grafico del mapa
map.plot();
% y trayectoria
veh.plot_xy('b');
% y trayectoria estimada por EKF
ekf.plot_xy('r');

% graficar elipse de covarianza cada 1 paso
ekf.plot_ellipse(10, 'm');

%% Plot de covarianza en el tiempo
figure
clf
ekf.plot_P();