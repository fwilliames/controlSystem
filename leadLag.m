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

%Lead Compensator Zeros and Poles
LEADzc = 10;%arbitrarily defined zero
    %Calculating the Angle to be compensated
    LeadG=(s + LEADzc)*2.7729/(s^2+0.717*s+9.985);
    Theta_s=(180/pi)*...
    angle(LeadG)
%     M=abs(G);
%     K=1/M;
Theta_s = - Theta_s;
theta = 180 - Theta_s

LEADpc = (wd/tand(theta)) + sigma %Calculating the Pole of the Compensator

%Transfer Function of the Lead part of the Lead-Lag Controller
cLead = tf([1 LEADzc],[1 LEADpc])

%System Roots Locus plus Lead part of Lead-Lag Controller -
%to define the gain
cLeadsys = series(cLead,sys);
figure(2)
rlocus(cLeadsys)
title('Lugar das Raizes do Sistema + Controlador de Avanço de Fase');
kLead = 534;
Leadfsys = feedback(kLead*cLeadsys,1);
figure(3)
step(Leadfsys,1)
title('Resposta ao Degrau do Sistema + Controlador de Avanço de Fase');

%Add the LAG part of the Lead-Lag controller. putting the pole
% arbitrarily close to the origin pc = 0.01 and the zero 10x away from the pole
%zc = 0.1
cLag = tf([1 0.1],[1 0.01]);
%System Roots Locus plus the LAG part of the Lead-Lag Controller -
%to define the gain 
cLeadLag = series(cLead,cLag);
cLeadLagsys = series(cLeadLag,sys);
figure(4)
rlocus(cLeadLagsys)
title('Lugar das Raizes do Sistema + Controlador de Avanço e Atraso de Fase');
kLag = 1515;

%Simulation of the system step response + System with Lead-Lag Controller

CLEAD = cLead;
CLAG = kLag*cLag;
CLEADLAG = series(CLEAD,CLAG);
CLeadLagsys = series(CLEADLAG,sys);
CLeadLagfsys = feedback(CLeadLagsys,1);
figure(5)
step(CLeadLagfsys)
title('Resposta ao Degrau do Sistema + Controlador de Avanço e Atraso de Fase');

figure(6)
step(fsys,12,'r')
hold on
step(CLeadLagfsys,'b')
title('Comparação entre a Resposta do Sistema e a Resposta do Sistema + Controlador de Avanço e Atraso de Fase')
hold off
