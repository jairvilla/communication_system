#******************************************************
# Aim: Código Matlab para modulación de Amplitud (AM)
# Course: Comunicaciones Análogas 
# Date: May, 03-21
# Author: Jair Villanueva
#******************************************************

% Esto es un ejemplo de un algoritmo p
clear all;
close all;

% Amplitud, Frecuencia y fase de señal modulante
A1 = 4; f1 = 5; p1 = 0;

% Amplitud, Frecuencia y fase de señal portadora.
A2 = 4; f2 = 20; p2 = 0;

% Frecuencia de muestreo de la señal 
fs = 1000; 

% Vector de tiempo. Longitud de la señal, será utilizada para la fft
t = 0: 1/fs : 1;

% Generar la señal de mensaje.
s1 = A1*sin(2*pi*f1*t + p1);

% Generación de la señal portadora 
c = A2*sin(2*pi*f2*t + p2);

%--------------------------
% Gráficas de la señal modulante y portadora
subplot (2,1,1), plot (t,s1);
xlabel('Tiempo (sec)');
ylabel('Amplitud');
title(['Señal modulante con Frecuencia= ',num2str(f1),' Hz']);
grid on;
Subplot (2,1,2), plot (t,c,'r');
xlabel('Tiempo (sec)');
ylabel('Amplitud');
title(['Señal portadora con Frecuencia= ',num2str(f2),' Hz']);
grid on;

%----------------------------
% Generación de DSB-SC) 
u = s1.*c; % Multiplicación punto a punto 
figure(1);
plot(t,u);
xlabel('Tiempo (sec)');
ylabel('Amplitud');
title(['Señal Modulada',' Hz']);
grid on;
% Generación de la envolvente 
s3_01= A1*A2*(sin(2*pi*f1*t));
s3_02= -A1*A2*(sin(2*pi*f1*t));
% Gráfica de la señal de DSB-SC
plot (t,u,'b'),
hold on
plot(t,s3_01,'r--');
hold on
plot (t,s3_02,'c--');
hold on 
xlabel('Tiempo(sec)');
ylabel('Amplitud');
title(['Señal Modulada DSB-SC',' Hz']);
hold off
%---------------------------
  % Generación de AM(DSB - FC) 
% SAM(t) = (1+ m(t))cos (2*pi*fc*t)
s4 = (A2 + s1).*sin(2*pi*f2*t);
% Generación de envolvente
s4_01 = A2 + s1;
s4_02 = -A2 - s1;
% Gráfica de AM(DSB - FC) 
figure(2);
plot (t,s4);
hold on
plot(t,s4_01,'r');
hold on;
plot(t,s4_02,'g');
hold on
xlabel('Tiempo (sec)');
ylabel('Amplitud');
title('Doble Banda lateral con portadora completa');
grid on;
hold off
%----------------------------
  % Número de puntos de FFT. N debe ser mayor que 
%la frecuencia de la portadora
N = 2^nextpow2(length(t));
f = fs * (0 : N/2) / N; 
% Find FFT
s3_f = (2/N)*abs(fft(u,N));
s4_f = (2/N)*abs(fft(s4,N));
%------------------------------
  % Gráficas de modulaciones DSB-SC y DSB-FC
figure(3);
subplot(2,2,1),
plot(t,u);
hold on
plot(t,s3_01,'r');
hold on;
plot(t,s3_02,'g');
hold off
xlabel('Tiempo (sec)');
ylabel('Amplitud');
title('Señal DSB-SC');
grid on;
%-------------------
  subplot (2,2,2),
plot (t,s4);
hold on
plot(t,s4_01,'r');
hold on;
plot(t,s4_02,'g');
hold on
xlabel('Tiempo (sec)');
ylabel('Amplitud');
title('Señal DSB-FC');
grid on;
%------------------
  subplot (2,2,3),
plot(f(1:100),s3_f(1:100));
xlabel('Frecuencia(Hz)');
ylabel('| Amplitud |');
title('Análisis espectral');
grid on;
%-------------------
  subplot (2,2,4),
plot(f(1:100),s4_f(1:100));
xlabel('Frecuencia(Hz)');
ylabel('| Amplitud |');
title('Análisis espectral');
grid on;
