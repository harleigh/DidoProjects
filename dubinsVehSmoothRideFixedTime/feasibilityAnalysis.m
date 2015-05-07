%{
    Feasibility Analysis for the our optimal control problem (minimizing a non-smooth ride
    in the Dubins Vehicle)

    As the file-name implies, here we perform feasability analysis upon the solution
    returned by DIDO.  Feasibility Analysis is a very important component of Verification
    and Validation of the results returned by DIDO.

    In Feasibility Analysis, we compare the extremal solution returned by DIDO to the
    solution Matlab returns from the system propagated using the extremal control returned
    by DIDO.
%}

clear all;
close all;

%results from the linear quadratic dido run.
load optCtrlPrimal;

%State is [x y theta], so now xOpt(1,:) is x over dido-time, xOpt(2,:) is y over didotime,
%and we have that xOpt(3,:) is theta over dido-time.
xOpt = primal.states;
uOpt = primal.controls;
tNodes = primal.nodes;

%Grab the start and end time used in linQuadProblemFile to produce the simulation which we
%are going to apply fieasability analysis upon.
t0 = tNodes(1);
tf = tNodes(end);


%The initial state fed into the ode solver is the DIDO output of the initial state
initState = xOpt(:,1);

%x is our state given as [x y theta] and u==omega our control.  The control for ode45
%is an interpolation of the control returned by DIDO.
u = @(t) interp1(tNodes, uOpt, t);

%Variable x is the state, so x(1)==x x(2)==y and x(3)==theta. The dynamics here parallels
%the dynamics file used for the dido-run.  Note 'v' was a global variable from the main
%problem file.
system = @(t, x) [ v*cos(x(3)); v*sin(x(3)); u(t)];

%Here is where we solve the ODE system using the control returned by DIDO, using the
%initial and final time retured by DIDO, and using the initial state from the extremals
%returned by DIDO.
[t, xOde45] = ode45( system, [t0 tf], initState);

%{
    Here we compare the results from DIDO  against the results
    returned from ode45 with the initial condition being the initial values of DIDO's
    numerical result.  We pass feasability if the solution from DIDO and the one from the
    propagation through ode45 match.

    The circles are the discrete points from the Dido run, and the solid line is the ode45
    result from the propagation of the dido control.
%}
figure;
subplot(3,1,1);
    plot(t,xOde45(:,1), tNodes, xOpt(1,:) ,'o');
    title('x vs t','fontSize',14,'fontWeight','bold');
    xlabel('t');
    ylabel('x   ');
    set(get(gca,'YLabel'),'Rotation',0)
subplot(3,1,2);
    plot(t,xOde45(:,2), tNodes, xOpt(2,:) ,'o');
    title('v vs t','fontSize',14,'fontWeight','bold');
    xlabel('t');
    ylabel('y   ');
    set(get(gca,'YLabel'),'Rotation',0)
subplot(3,1,3);
    plot(t,xOde45(:,3), tNodes, xOpt(3,:) ,'o');
    title('v vs t','fontSize',14,'fontWeight','bold');
    xlabel('t');
    ylabel('y   ');
    set(get(gca,'YLabel'),'Rotation',0)
    
%
% End feasibilityAnalysis.m
%