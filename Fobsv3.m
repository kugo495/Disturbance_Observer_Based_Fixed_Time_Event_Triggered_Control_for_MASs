function [sys,x0,str,ts,simStateCompliance] = Fobsv3(t,x,u,flag)

switch flag,

  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

  case 1,
    sys=mdlDerivatives(t,x,u);

  case 2,
    sys=mdlUpdate(t,x,u);

  case 3,
    sys=mdlOutputs(t,x,u);

  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  case 9,
    sys=mdlTerminate(t,x,u);

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
sizes = simsizes;

sizes.NumContStates  = 2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 16;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = [0 0];

str = [];

ts  = [0 0];

simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u)
z1=[u(3)-u(1);u(5)-u(1);x(1)-u(1);u(7)-u(1);u(9)-u(1)];
z2=[u(4)-u(2);u(6)-u(2);x(2)-u(2);u(8)-u(2);u(10)-u(2)];
L = [2 -1 -1 0 0;-1 2 0 -1 0;-1 0 3 -1 -1;0 -1 -1 2 0;0 0 -1 0 1];
B = diag([1,0,0,0,1]);
mu = u(11); Tc = u(12); neta = u(13); w1 = u(14); w2 = u(15); gama_n = u(16);
f1 = 0.8*x(1)*sin(x(1)*x(1));
f2 = 0.8*x(1)*x(2)*x(2);
p1 = -1/(0.2841^(1-mu/2));
p2 = -1/(0.2841^(1-mu/2));
phi1 = -1/(0.2841^(1+mu/2));
phi2 = -1/(0.2841^(1+mu/2));
m1 = (pi/(mu*Tc*neta))*(p1*(0.5^(1-mu/2))*sign((L+B)*z1).*(abs((L+B)*z1)).^(1-mu)+phi1*(0.5^(1+mu/2))*(5^(mu/2))*(neta^2)*sign((L+B)*z1).*(abs((L+B)*z1)).^(1+mu))+w1*diag(sign((L+B)*z1))*ones(5,1)*norm((L+B)*z1,1);
sys(1) = m1(3,:)+f1+x(2);
m2 = (pi/(mu*Tc*neta))*(p2*(0.5^(1-mu/2))*sign((L+B)*z2).*(abs((L+B)*z2)).^(1-mu)+phi2*(0.5^(1+mu/2))*(5^(mu/2))*(neta^2)*sign((L+B)*z2).*(abs((L+B)*z2)).^(1+mu))+gama_n*sign((L+B)*z2)+w2*diag(sign((L+B)*z2))*ones(5,1)*(norm((L+B)*z1,1)+norm((L+B)*z2,1));
sys(2) = m2(3,:)+f2;

function sys=mdlUpdate(t,x,u)

sys = [];

function sys=mdlOutputs(t,x,u)
sys(1) = x(1);
sys(2) = x(2);

function sys=mdlGetTimeOfNextVarHit(t,x,u)
sys=[];

function sys=mdlTerminate(t,x,u)
sys = [];

