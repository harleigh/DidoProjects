%{
    Feasibility Analysis for the linear Quadratic problem

    Assumptions: The Linear Quadratic Problem has been run by DIDO (i.e.:
    linQuadProblemFile.m has been ran)

    As the file-name implies, here we perform feasability analysis upon the solution
    returned by DIDO.  Feasibility Analysis is a very important component of Verification
    and Validation of the results returned by DIDO.

    In Feasibility Analysis, we compare the extremal solution returned by DIDO to the
    solution Matlab returns from the system propagated using the extremal control returned
    by DIDO.
%}

clear all;
close all;

%results from the linear quadratic dido run. Variables used are tNodes, uOpt, xOpt, vOpt
load linQuadPrimal;


xOpt = primal.states(1,:);
vOpt = primal.states(2,:);
uOpt = primal.controls;
tNodes = primal.nodes;

%Grab the start and end time used in linQuadProblemFile to produce the simulation which we
%are going to apply fieasability analysis upon.
t0 = tNodes(1);
tf = tNodes(end);

%The initial state fed into the ode solver is the DIDO output of the initial state
initState = [xOpt(1);vOpt(1)];

%x is our state given as [x v] and u our control.  The control is an interpolation of the
%control returned by DIDO.  We know from the output of DIDO, the control is quite linear,
%so calling interp1 as given works well.  We also know, from PMP, that the extremal
%control necessarily Must be linear.
u = @(t) interp1(tNodes, uOpt, t);
system = @(t, x) [ x(2); u(t)];

%Here is where we solve the ODE system using the control returned by DIDO, using the
%initial and final time retured by DIDO, and using the initial state from the extremals
%returned by DIDO.
[t, x] = ode45( system, [t0 tf], initState);

%{
    Here we compare the results from DIDO (stored in xOpt and vOpt) against the results
    returned from ode45 with the initial condition being the initial values of DIDO's
    numerical result.  We pass feasability if the solution from DIDO and the one from the
    propagation through ode45 match.
%}
figure;
subplot(2,1,1);
    plot(t,x(:,1), tNodes, xOpt ,'o');
    title('x vs t','fontSize',14,'fontWeight','bold');
    xlabel('t');
    ylabel('x   ');
    set(get(gca,'YLabel'),'Rotation',0)
subplot(2,1,2);
    plot(t,x(:,2), tNodes, vOpt ,'o');
    title('v vs t','fontSize',14,'fontWeight','bold');
    xlabel('t');
    ylabel('v   ');
    set(get(gca,'YLabel'),'Rotation',0)
    
%
% End feasibilityAnalysis.m
%