%{
    Feasibility Analysis for the Brachistochrone Problem

    Assumptions: The optimal control Problem has been run by DIDO (i.e.:
    mainProblemFile.m has been ran)

    As the file-name implies, here we perform feasability analysis upon the solution
    returned by DIDO.  Feasibility Analysis is a very important component of Verification
    and Validation of the results returned by DIDO.

    In Feasibility Analysis, we compare the extremal solution returned by DIDO to the
    solution Matlab returns from the system propagated using the extremal control returned
    by DIDO.
%}

clear all;
close all;

%results from the dido run. Variables used are tNodes, uOpt, xOpt, vOpt
load terminalPointPrimal;

%Grab the start and end time used in linQuadProblemFile to produce the simulation which we
%are going to apply fieasability analysis upon.
t0 = tNodes(1);
tf = tNodes(end);

%The initial state fed into the ode solver is the DIDO output of the initial state
initState = [xOpt(1);yOpt(1); vOpt(1)];


uXdir = @(t) interp1(tNodes, uOpt(1,:), t);
uYdir = @(t) interp1(tNodes, uOpt(2,:), t);
%Here, x is our state vector: x(1) is x, x(2) is y and x(3) is v.  Note 'g' is defined in
%the main file
system = @(t, x) [ x(3)*uXdir(t); x(3)*uYdir(t); 9.8*uYdir(t)];

%Here is where we solve the ODE system using the control returned by DIDO, using the
%initial and final time retured by DIDO, and using the initial state from the extremals
%returned by DIDO.
[t, x] = ode45( system, [t0 tf], initState);

%{
    Here we compare the results from DIDO against the results
    returned from ode45 with the initial condition being the initial values of DIDO's
    numerical result.  We pass feasability if the solution from DIDO and the one from the
    propagation through ode45 match.
%}
figure;
subplot(3,1,1);
    plot(t,x(:,1), tNodes, xOpt ,'o');
    title('x vs t','fontSize',14,'fontWeight','bold');
    xlabel('t');
    ylabel('x   ');
    set(get(gca,'YLabel'),'Rotation',0)
subplot(3,1,2);
    plot(t,x(:,2), tNodes, yOpt ,'o');
    title('y vs t','fontSize',14,'fontWeight','bold');
    xlabel('t');
    ylabel('y   ');
    set(get(gca,'YLabel'),'Rotation',0);
subplot(3,1,3);
    plot(t,x(:,3), tNodes, vOpt ,'o');
    title('v vs t','fontSize',14,'fontWeight','bold');
    xlabel('t');
    ylabel('v   ');
    set(get(gca,'YLabel'),'Rotation',0)
    
%
% End feasibilityAnalysis.m
%