div=@()disp("-------------------");
%% use a pendulum with a masspoint at the end of the stick

g=9.81;
m=0.15;
b=0.1;
L=0.5;

controlled_pendulum_dynamics_with_friction = @(t, x, u, L, g, m, b) ...
    [x(2); (m * g * L * sin(x(1)) - b * x(2) + u) / ( m * L^2)];

%% linearize using jacobian to get LQR ontrol 
A_parametric=@(g,m,b,L) ...
    [0 1;...
    g/L -b];
A=A_parametric(g,m,b,L) ;


B_parametric=@(g,m,b,L)...
    [0;...
    1/m*L*L];
B=B_parametric(g,m,b,L);

div()
disp("A=")
disp(A_parametric)
disp([A])
div()
disp("B=")
disp(B_parametric)
disp([B])
div()


%% calculate lqr
Q=diag([1,1]);
R=1;

k=lqr(A,B,Q,R);

disp("K=")
disp(k)
div()

%% Simulate open loop
tspan=[0 10];
theta0=0.1;theta_dot0=0;

figure(1)
[t,y]=ode45(@(t, y) controlled_pendulum_dynamics_with_friction(t, y, 0, L, g, m, b), tspan, [theta0, theta_dot0]);
plot(t,y)
legend("theta","theta-dot")
title("NonlinearOpenLoop")


figure(2)
[t,y]=ode45(@(t, y) A*y+B*(0), tspan, [theta0, theta_dot0]);
plot(t,y)
legend("theta","theta-dot")
title("LinearOpenLoop")




figure(3)
[t,y]=ode45(@(t, y) controlled_pendulum_dynamics_with_friction(t, y, -k*y, L, g, m, b), tspan, [theta0, theta_dot0]);
tiledlayout(2,1)
nexttile
plot(t,y)
legend("theta","theta-dot")
nexttile
plot(t,k*y')
legend("u[Nm]")
title("NonLinearLQRControl")





figure(4)
[t,y]=ode45(@(t, y) A*y+B*(-k*y), tspan, [theta0, theta_dot0]);
tiledlayout(2,1)
nexttile
plot(t,y)
legend("theta","theta-dot")
nexttile
plot(t,k*y')
legend("u[Nm]")
title("NonLinearLQRControl")

title("LinearLQRControl")





