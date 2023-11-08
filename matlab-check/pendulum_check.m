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
    1/(m*L*L)];
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
title("LinearLQRControl")




%% Learned control 
ly_controller=@(x1,x2)tanh((2.4234895265889431 + 1.8116612736218094 * tanh((-1.699560669266909 - 0.063405064273362358 * x1 + 0.32606535601543257 * x2)) + 1.8977886097975294 * tanh((-1.1270676920679739 + 0.26177797528889291 * x1 - 0.079115557548884441 * x2)) + 1.2639467038182315 * tanh((-0.4904079564875039 - 0.26230984812802949 * x1 - 0.43826484737155086 * x2)) + 1.0945367622073336 * tanh((0.28702417546834091 + 0.10149126322478071 * x1 + 0.42627845129313652 * x2)) + 3.0898100164277094 * tanh((2.435036688696941 - 0.024031367582632239 * x1 - 0.00067241062872665854 * x2)) - 1.9197170122697864 * tanh((2.5272624876394865 + 0.3653448050903369 * x1 + 0.25222104817227281 * x2))));

figure(5)
[t,y]=ode45(@(t, y) controlled_pendulum_dynamics_with_friction(t, y, ly_controller(y(1),y(2)), L, g, m, b), tspan, [theta0, theta_dot0]);
tiledlayout(2,1)
nexttile
plot(t,y)
legend("theta","theta-dot")
nexttile
plot(t,ly_controller(y(:,1),y(:,2)))
legend("u[Nm]")
title("NonLinearLQRControl")




