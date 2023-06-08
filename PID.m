%System Transfer Function
num = [2.7925];
den = [1 0.717 9.9849];
sys = tf(num,den);
fsys = feedback(sys,1);
figure(1)
rlocus(sys)
title('Lugar das Raizes do Sistema Nao Compensado');

%Project Parameters Mp = 10%; Ts = 0.5s
mp = 0.10;
ts = 0.5;
damp = 0.5912; %Defining damping factor
wn = 13.5318; %Defining the undamped natural frequency

sigma = damp * wn; %Defining sigma, real part of the desired pole
wd = wn * sqrt(1 - damp^2); %Defining imaginary part of the desired pole

% Desired closed-loop poles
s = -sigma+wd*1i
s2 = -sigma-wd*1i;

%Zero of the PD Controller
PDG=2.7925/(s^2+0.7170*s+9.9849);
theta_s=(180/pi)*...
angle(PDG)
%M=abs(G)
%K=1/M;
theta = 180 - theta_s
PDzc = sigma + (wd/tand(theta))

% Transfer Function of the derivative part of the PID Controller
cpd = tf([1 PDzc],[1])

%System Roots Locus plus the derivative part of the PID Controller -
%to define the gain 
cpdsys = series(cpd,sys);
figure(2)
rlocus(cpdsys)
title('Lugar das Raizes do Sistema + Controlador PD');

kd = 25;
figure(3)
step(feedback((kd*cpdsys),1))
title('Resposta ao degrau do Sistema + Controlador PD');

%Add the integral part of the PID controller. putting the zero
% arbitrarily close to the origin zc = 0.05
PIzc = 0.05;
cpi = tf([1 PIzc],[1 0]);

% Place of the System Roots plus the integrative part of the PID Controller -
%to define the gain 
cpisys = series(cpi,cpdsys);
figure(4)
rlocus(cpisys)
title('Lugar das Raizes do Sistema + Controlador PDI');
ki = 5.49;

%Calculating K1 and K2
K_3 = 20
K_1 = K_3*(PDzc + PIzc)
K_2 = K_3*PDzc*PIzc

CPD = tf([1 0],1);
CPI = tf(1,[1 0]);

CPID = K_1 + K_2*CPI+ K_3*CPD

CPIDsys = series(CPID,sys);
CPIDfsys = feedback(CPIDsys,1);
figure(5)
step(CPIDfsys,'b')
title('Resposta ao degrau do Sistema + Controlador PID');


figure(6)
step(fsys,12,'r')
hold on
step(feedback(CPIDsys,1),'g')
hold off
title('Comparacao entre Resposta do Sistema e a Resposta do Sistema + Controlador PID');
