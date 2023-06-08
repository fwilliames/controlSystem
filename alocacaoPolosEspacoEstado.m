%System Transfer Function
num = [2.7925];
den = [1 0.7170 9.9849];
sys = tf(num,den);
fsys = feedback(sys,1);
step(fsys,'g')
title('Resposta ao Degrau do Sistema Pendulo-Helice');

%Modelling of the System in State Space
a2 = 9.9849;
a1 = 0.7170;
b2 = 2.7925;

A = [0 1; -a2 -a1];
B = [0; 1];
C = [b2 0];
D = [0];
K = [1 1];

%Project Parameters Mp = 10%; Ts = 0.5s
mp = 0.10;
ts = 0.5;
damp = 0.5912; %Defining damping factor
wn = 13.5318; %Defining the undamped natural frequency

sigma = damp * wn; %Defining sigma, real part of the desired pole
wd = wn * sqrt(1 - damp^2); %Defining imaginary part of the desired pole

% Desired closed-loop poles
p1 = -sigma+wd*1i
p2 = -sigma-wd*1i

% Desired Characteristic Equation s^3 + d2s^2 + d1s + d0
p1 = tf([1 8+10.9137*1i],[1]);
p2 = tf([1 8-10.9137*1i],[1]);
p3 = tf([1 40],[1]);
eqc = p1*p2*p3

d0 = 7324;
d1 = 823.1;
d2 = 56;

%Defining k1 k2 and ke
k1 = d1 - a2
k2 = d2 - a1
ke = d0/b2

%System + Integrator + Pole Assignment Controller
AA = [0 1 0; -(k1+a2) -(k2+a1) ke; -2.7729 0 0];
BB = [0; 0; 1];
CC = [2.7729 0 0];

%Response to the Step of the System + Integrator + Pole Assignment Controller
figure(1)
step(AA,BB,CC,D,1,'b')
title('Simulacao do Sistema Pendulo-Helice no Espaco de Estado + Controlador por Alocacao de Polos');

figure(2)
step(fsys,'r')
hold on
step(AA,BB,CC,D,1,'y')
hold off
title('Comparacao da Resposta do Sistema e a Resposta do Sistema + Controlador por Alocacao de Polos');