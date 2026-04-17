function [sys,x0,str,ts,simStateCompliance] = follower3(t,x,u,flag)

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
sizes.NumInputs      = 5;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = [0.001 0];

str = [];

ts  = [0 0];

simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u)
f1 = 0.8*x(1)*sin(x(1)*x(1));
f2 = 0.8*x(1)*x(2)*x(2);
uu = u(1); m = u(2); r = u(3); d1 = u(4); d2 = u(5);
sys(1) = f1+x(2)+d1;
sys(2) = f2+(m*uu+r)+d2;

function sys=mdlUpdate(t,x,u)

sys = [];

function sys=mdlOutputs(t,x,u)
sys(1) = x(1);
sys(2) = x(2);

function sys=mdlGetTimeOfNextVarHit(t,x,u)
sys=[];

function sys=mdlTerminate(t,x,u)
sys = [];

