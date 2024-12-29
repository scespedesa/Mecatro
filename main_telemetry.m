close all; clear all; clc;


%WiFi settings
IP = '192.168.4.1';
PORT_NUMBER = 80;

%USB settings
port_name = '/dev/cu.usbmodemF412FA70B6442';
baudrate = 230400;

%Recording time
Tmax = 50;

%Communication method
method = 'WiFi';

%Gains
K = [];

if strcmp(method,'WiFi')
    [log_time, data_values, line_idx] = get_data_WiFi(IP, PORT_NUMBER, Tmax, K);
elseif strcmp(method,'USB')
    [log_time, data_values, line_idx] = get_data_USB(port_name,Tmax,baudrate, K);
else
    print('Use a valid connection method')
end

%Display logged variable names
disp(data_values.keys)  

% Plot logged variables
figure; 
hold on;
legend_labels = data_values.keys;
for i = 1:numel(legend_labels)
    d = legend_labels{i};
    v = data_values(d);
    plot(log_time, v, 'DisplayName', d);
    title(d); % Opcional: Agrega un título con el nombre de la serie de datos
    xlabel('Log Time'); % Opcional: Etiqueta del eje x
    ylabel('Value'); % Opcional: Etiqueta del eje y
    legend show; % Muestra la leyenda
    %plot(log_time, v, 'DisplayName', d);
end
hold off;
legend('Location', 'best');
grid on;
% Factor de conversión de kgf.mm a Nm
factor_conversion = 9.80665e-3;
k_couple=144*factor_conversion/(5.6-0.15);

%% Obtencion de las funciones de transferencia angulos y velocidades motores

% Datos de entrada y salida (asegúrate de que estén alineados en tamaño)
y1 = data_values('Vr_f1');   % Datos de salida (posición del motor)
u1 = data_values('Ur');    % Datos de entrada (voltaje aplicado al motor)
Ts = mean(diff(log_time));     % Calcula el tiempo de muestreo promedio entre puntos

% Crear objeto iddata
data = iddata(y1', u1', Ts);

% Número de ceros y polos para el modelo de motor
num_zeros = 0;   % Número de ceros (un sistema simple de motor puede no tener ceros)
num_poles = 1;   % Número de polos 

% Estimar función de transferencia
sys1 = tfest(data, num_poles, num_zeros);

% Mostrar la función de transferencia estimada
disp(sys1);

% Datos de entrada y salida (asegúrate de que estén alineados en tamaño)
y2 = data_values('Vl_f1');   % Datos de salida (posición del motor)
u2 = data_values('Ul');    % Datos de entrada (voltaje aplicado al motor)
%Ts = mean(diff(log_time));     % Calcula el tiempo de muestreo promedio entre puntos

% Crear objeto iddata
data = iddata(y2', u2', Ts);

% Número de ceros y polos para el modelo de motor
num_zeros = 0;   % Número de ceros (un sistema simple de motor puede no tener ceros)
num_poles = 1;   % Número de polos (un sistema de segundo orden)

% Estimar función de transferencia
sys2 = tfest(data, num_poles, num_zeros);

% Mostrar la función de transferencia estimada
disp(sys2);


% Datos de entrada y salida (asegúrate de que estén alineados en tamaño)
y3 = data_values('Ar');   % Datos de salida (posición del motor)
u3 = data_values('Ur');    % Datos de entrada (voltaje aplicado al motor)
Ts = mean(diff(log_time));     % Calcula el tiempo de muestreo promedio entre puntos

% Crear objeto iddata
data = iddata(y3', u3', Ts);

% Número de ceros y polos para el modelo de motor
num_zeros = 0;   % Número de ceros (un sistema simple de motor puede no tener ceros)
num_poles = 2;   % Número de polos (un sistema de segundo orden)

% Estimar función de transferencia
sys3 = tfest(data, num_poles, num_zeros);

% Mostrar la función de transferencia estimada
disp(sys3);

% Datos de entrada y salida (asegúrate de que estén alineados en tamaño)
y4 = data_values('Al');   % Datos de salida (posición del motor)
u4 = data_values('Ul');    % Datos de entrada (voltaje aplicado al motor)
%Ts = mean(diff(log_time));     % Calcula el tiempo de muestreo promedio entre puntos

% Crear objeto iddata
data = iddata(y4', u4', Ts);

% Número de ceros y polos para el modelo de motor
num_zeros = 0;   % Número de ceros (un sistema simple de motor puede no tener ceros)
num_poles = 2;   % Número de polos (un sistema de segundo orden)

% Estimar función de transferencia
sys4 = tfest(data, num_poles, num_zeros);

% Mostrar la función de transferencia estimada
disp(sys4);



%% Obtencion de las funciones de transferencia sistema global
% Datos de entrada y salida 
y5 = data_values('delta_u');   % Datos de salida (posición del motor)
u5 = data_values('U_plus');    % Datos de entrada (voltaje aplicado al motor)
Ts = mean(diff(log_time));     % Calcula el tiempo de muestreo promedio entre puntos

% Crear objeto iddata
data = iddata(y5', u5', Ts);

% Número de ceros y polos para el modelo de motor
num_zeros = 0;   % Número de ceros (un sistema simple de motor puede no tener ceros)
num_poles = 1;   % Número de polos (un sistema de segundo orden)

% Estimar función de transferencia
sys5 = tfest(data, num_poles, num_zeros);

disp(sys5);

% Datos de entrada y salida (asegúrate de que estén alineados en tamaño)
y6 = data_values('delta_r');   % Datos de salida (posición del motor)
u6 = data_values('U_moins');    % Datos de entrada (voltaje aplicado al motor)

% Crear objeto iddata
data = iddata(y6', u6', Ts);

% Número de ceros y polos para el modelo de motor
num_zeros = 0;   % Número de ceros (un sistema simple de motor puede no tener ceros)
num_poles = 1;   % Número de polos

% Estimar función de transferencia
sys6 = tfest(data, num_poles, num_zeros);

disp(sys6);
%% Con 2 filtros

% Datos de entrada y salida 
y7 = data_values('Vr_f2');   % Datos de salida (posición del motor)
u7 = data_values('Ur');    % Datos de entrada (voltaje aplicado al motor)
Ts = mean(diff(log_time));     % Calcula el tiempo de muestreo promedio entre puntos

% Crear objeto iddata
data = iddata(y7', u7', Ts);

% Número de ceros y polos para el modelo de motor
num_zeros = 0;   % Número de ceros (un sistema simple de motor puede no tener ceros)
num_poles = 1;   % Número de polos (un sistema de segundo orden)

% Estimar función de transferencia
sys7 = tfest(data, num_poles, num_zeros);

% Mostrar la función de transferencia estimada
disp(sys7);

% Datos de entrada y salida (asegúrate de que estén alineados en tamaño)
y8 = data_values('Vl_f2');   % Datos de salida (posición del motor)
u8 = data_values('Ul');    % Datos de entrada (voltaje aplicado al motor)
%Ts = mean(diff(log_time));     % Calcula el tiempo de muestreo promedio entre puntos

% Crear objeto iddata
data = iddata(y8', u8', Ts);

% Número de ceros y polos para el modelo de motor
num_zeros = 0;   % Número de ceros (un sistema simple de motor puede no tener ceros)
num_poles = 1;   % Número de polos (un sistema de segundo orden)

% Estimar función de transferencia
sys8 = tfest(data, num_poles, num_zeros);

% Mostrar la función de transferencia estimada
disp(sys8);
%% si fuera de 2 orden

% Datos de entrada y salida 
y10 = data_values('delta_r');   % Datos de salida (posición del motor)
u10 = data_values('U_moins');    % Datos de entrada (voltaje aplicado al motor)

% Crear objeto iddata
data = iddata(y10', u10', Ts);

% Número de ceros y polos para el modelo de motor
num_zeros = 0;   % Número de ceros (un sistema simple de motor puede no tener ceros)
num_poles = 2;   % Número de polos (un sistema de segundo orden)

% Estimar función de transferencia
sys10 = tfest(data, num_poles, num_zeros);

disp(sys10);

%% Comparativa funcion velocidad con dos filtros

% Plot logged variables
figure(10); 
hold on;
plot(log_time, data_values('Vr_f1'),'DisplayName', 'Vr');
plot(log_time, data_values('Vr_f2'),'DisplayName', 'Vr-deux-filtre');
xlabel('Log Time'); % Opcional: Etiqueta del eje x
ylabel('Value'); % Opcional: Etiqueta del eje y
legend show; % Muestra la leyenda
hold off; 
legend('Location', 'best');
grid on;

figure(11); 
hold on;
plot(log_time, data_values('Vl_f1'),'DisplayName', 'f1');
plot(log_time, data_values('Vl_f2'),'DisplayName', 'Vl-deux-filtre');
xlabel('Log Time'); % Opcional: Etiqueta del eje x
ylabel('Value'); % Opcional: Etiqueta del eje y
legend show; % Muestra la leyenda
hold off; 
legend('Location', 'best');
grid on;


%% resultados
% Plot logged variables
figure(4); 
hold on;
plot(log_time, data_values('U_plus'),'DisplayName', 'u-plus');
plot(log_time, data_values('delta_u'),'DisplayName', 'delta-u');
title('Performance de la vitesse de translation de la voiture'); % Opcional: Agrega un título con el nombre de la serie de datos
xlabel('Temps'); % Opcional: Etiqueta del eje x
ylabel('Vitesse de la voiture [m/s]'); % Opcional: Etiqueta del eje y
legend show; % Muestra la leyenda
hold off; 
legend('Location', 'best');
grid on;

figure(5); 
hold on;
plot(log_time, data_values('U_moins'),'DisplayName', 'U-moins');
plot(log_time, data_values('delta_r'),'DisplayName', 'delta-r');
title('Performance de la vitesse de rotation de la voiture'); % Opcional: Agrega un título con el nombre de la serie de datos
xlabel('Temps'); % Opcional: Etiqueta del eje x
ylabel('Vitesse du moteur droit [rad/s]'); % Opcional: Etiqueta del eje y
legend show; % Muestra la leyenda
hold off; 
legend('Location', 'best');
grid on;

figure(6); 
hold on;
plot(log_time, data_values('Ur'),'DisplayName', 'Ur');
plot(log_time, data_values('Vr_f2'),'DisplayName', 'Vr-deux-filtre');
title('Réponse du moteur à divers signaux de tension'); 
xlabel('Temps'); % Opcional: Etiqueta del eje x
ylabel('Vitesse du moteur droit [rad/s]'); % Opcional: Etiqueta del eje y
legend show; % Muestra la leyenda
hold off; 
legend('Location', 'best');
grid on;


figure(7); 
hold on;
plot(log_time, data_values('Ul'),'DisplayName', 'Ul');
plot(log_time, data_values('Vl_f2'),'DisplayName', 'Vl-deux-filtre');
title('Réponse du moteur à divers signaux de tension');
xlabel('Temps'); % Opcional: Etiqueta del eje x
ylabel('Vitesse du moteur gauche [rad/s]'); % Opcional: Etiqueta del eje y
legend show; % Muestra la leyenda
hold off; 
legend('Location', 'best');
grid on;


figure(8); 
hold on;
plot(log_time, data_values('Ul'),'DisplayName', 'Ul');
plot(log_time, data_values('Al'),'DisplayName', 'Al');
title('Réponse du moteur à divers signaux de tension');
xlabel('Temps'); % Opcional: Etiqueta del eje x
ylabel('Position angulaire du moteur gauche [rad]'); % Opcional: Etiqueta del eje y
legend show; % Muestra la leyenda
hold off; 
legend('Location', 'best');
grid on;

figure(9); 
hold on;
plot(log_time, data_values('Ur'),'DisplayName', 'Ur');
plot(log_time, data_values('Ar'),'DisplayName', 'Ar');
title('Réponse du moteur à divers signaux de tension');
xlabel('Temps'); % Opcional: Etiqueta del eje x
ylabel('Position angulaire du moteur droit [rad]'); % Opcional: Etiqueta del eje y
legend show; % Muestra la leyenda
hold off; 
legend('Location', 'best');
grid on;


%% grafica

% Parámetros iniciales
x0 = 0; % Posición inicial en x
y0 = 0; % Posición inicial en y
theta0 = 0; % Ángulo inicial (en radianes)

% Suponiendo que tienes w, v y t definidos:
w= data_values('delta_r');
v=data_values('delta_u');
w=w(1:3951);
v=v(1:3951);
t = log_time(1:3951);
% t - vector de tiempos (s)

% Inicializa los vectores para almacenar la posición
x = zeros(length(t), 1);
y = zeros(length(t), 1);
theta = zeros(length(t), 1);

% Condiciones iniciales
x(1) = x0;
y(1) = y0;
theta(1) = theta0;

% Método de integración (Euler) para obtener la trayectoria
for i = 2:length(t)
    dt = t(i) - t(i-1); % Intervalo de tiempo
    
    % Actualiza el ángulo con la velocidad angular
    theta(i) = theta(i-1) + w(i-1) * dt;
    
    % Actualiza la posición con la velocidad de traslación y el ángulo
    x(i) = x(i-1) + v(i-1) * cos(theta(i-1)) * dt;
    y(i) = y(i-1) + v(i-1) * sin(theta(i-1)) * dt;
end

% Graficar la trayectoria
figure;
plot(x, y, '-o');
xlabel('Position X (m)');
ylabel('Position Y (m)');
title('trajectoire du robot avec les poids');
grid on;
axis equal;
%% resultados pista
% Plot logged variables
figure(4); 
hold on;
plot(log_time, data_values('U_plus'),'DisplayName', 'u-plus');
plot(log_time, data_values('delta_u'),'DisplayName', 'delta-u');
title('Performance de la vitesse de déplacement de la voiture sur le parcours'); % Opcional: Agrega un título con el nombre de la serie de datos
xlabel('Temps'); % Opcional: Etiqueta del eje x
ylabel('Vitesse de la voiture [m/s]'); % Opcional: Etiqueta del eje y
legend show; % Muestra la leyenda
hold off; 
legend('Location', 'best');
grid on;

figure(5); 
hold on;
plot(log_time, data_values('U_moins'),'DisplayName', 'U-moins');
plot(log_time, data_values('delta_r'),'DisplayName', 'delta-r');
title('Performance de la vitesse de rotation sur le parcours'); % Opcional: Agrega un título con el nombre de la serie de datos
xlabel('Temps'); % Opcional: Etiqueta del eje x
ylabel('Vitesse de la voiture [rad/s]'); % Opcional: Etiqueta del eje y
legend show; % Muestra la leyenda
hold off; 
legend('Location', 'best');
grid on;

figure(6); 
hold on;
plot(log_time, data_values('Ur')*5,'DisplayName', 'Ur');
plot(log_time, data_values('Vr_f1'),'DisplayName', 'Vr');
title('Réponse du moteur sur le parcours'); 
xlabel('Temps'); % Opcional: Etiqueta del eje x
ylabel('Vitesse du moteur droit [rad/s]'); % Opcional: Etiqueta del eje y
legend show; % Muestra la leyenda
hold off; 
legend('Location', 'best');
grid on;


figure(7); 
hold on;
plot(log_time, data_values('Ul')*5,'DisplayName', 'Ul');
plot(log_time, data_values('Vl_f1'),'DisplayName', 'Vl');
title('Réponse du moteur sur le parcours');
xlabel('Temps'); % Opcional: Etiqueta del eje x
ylabel('Vitesse du moteur gauche [rad/s]'); % Opcional: Etiqueta del eje y
legend show; % Muestra la leyenda
hold off; 
legend('Location', 'best');
grid on;


figure(8); 
hold on;
plot(log_time, data_values('Ul')*10,'DisplayName', 'Ul');
plot(log_time, data_values('Al'),'DisplayName', 'Al');
title('Réponse du moteur selon la tension sur le parcours');
xlabel('Temps'); % Opcional: Etiqueta del eje x
ylabel('Position angulaire du moteur gauche [rad]'); % Opcional: Etiqueta del eje y
legend show; % Muestra la leyenda
hold off; 
legend('Location', 'best');
grid on;

figure(9); 
hold on;
plot(log_time, data_values('Ur')*10,'DisplayName', 'Ur');
plot(log_time, data_values('Ar'),'DisplayName', 'Ar');
title('Réponse du moteur selon la tension sur le parcours');
xlabel('Temps'); % Opcional: Etiqueta del eje x
ylabel('Position angulaire du moteur droit [rad]'); % Opcional: Etiqueta del eje y
legend show; % Muestra la leyenda
hold off; 
legend('Location', 'best');
grid on;
%% guardar datos


% Crear la tabla con los datos a partir de los vectores en el workspace
T = table(log_time', data_values('Ur')', data_values('Ul')', data_values('Vr_f2')', data_values('Vl_f2')', data_values('Ar')',data_values('Al')','VariableNames', {'Time', 'Ur', 'Ul', 'Vr_f2', 'Vl_f2','Ar','Al'});

% Guardar la tabla en un archivo CSV llamado 'data_log.csv'
writetable(T, 'data_Hermoso2-conangulos-rotando.csv');

% Confirmación opcional: mostrar un mensaje cuando se complete el guardado
disp('Datos guardados exitosamente en ');

%% PID


% Parámetros de las funciones de transferencia
a = 3.585;
b = 5.989;
c = 92.86;
d = 21.74; 
%11.47
%20.93
%3.585;
%5.989;
%12.51
%22.27

%93.16
%20.67

% Definición de la variable Laplace
s = tf('s');

% Funciones de transferencia
G_velocidadTraslacion = a / (s + b);
G_velocidadAngular = c / (s + d);
G_velocidadAngular1=2.696e04/(s^2 + 288.1*s + 6325);

%2.696e04
%288.1
%6325

%2.777e04
%s^2 + 295.8*s + 6353
% PID para la velocidad de traslación
pidTraslacion = pidTuner(G_velocidadTraslacion, 'PID');
% PID para la velocidad angular
pidAngular = pidTuner(G_velocidadAngular, 'PID');

pidAngular1 = pidTuner(G_velocidadAngular1, 'PID');

% Visualización de los parámetros del PID
% Una vez ajustados en pidTuner, puedes usar los valores de Kp, Ki, y Kd.

