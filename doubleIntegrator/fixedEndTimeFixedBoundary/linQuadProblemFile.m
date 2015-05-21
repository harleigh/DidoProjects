%{
    Using Dido, we solve the Linear Quadratic Problem in section 3.3 in Michael Ross' text
    "A primer on Pontryagin's Princople in Optimal Control"

    State x v, where x is position, v is velocity
        Control u represents force, is bounded by -1 and 1 for all time
        Minimize quadratic running cost integrate from t0 to tf u^2
        Dynamics  dx = v and dv = u  (where d(.) is time derivative) Dynamics come from
            force=mass*acceleration
        Start at a given time zero: t0=0
        Start at a given time tf
        Object starts with a given position at rest
        Object must end at position zero at rest

    Verification and Validtation are given at the end of this file. We compare the reuslts
    from Dido to the Analytical results derived from Pontryagin's Minimization Principal.

    See attached PDF for problem statement.
%}
clear all; close all;

Nn = 128;
makeGuess = 0;
loadGuess = 1;
runAccruateMode = 0;

%Setting the box constraints to the Linear Quadratic problem.  Note that setting the box
%costraints determines the order of the state vectors.  Hence row 1 is x and row 2 is v.
%Careful when setting the box constraints: (1) Watch out for singularities in the box
%constraints (2) Watch out for setting them too large (3) Watch out for setting them too
%small :) ...It can be difficult sometimes.
xL = -1; xU = 10;
vL = -1; vU = 10;
bounds.lower.states = [xL; vL];
bounds.upper.states = [xU; vU];


%set the box constraints for the control u. Note: u is unconstrained.  Make
%large so it's unbinding, but not too large (or numerical issues ensue)
uL = -10; uU = 10;
bounds.lower.controls = [uL];
bounds.upper.controls = [uU];


%set the bounds for the initial and final time.  Initial time and final time are
%fixed at 0 and 1 respectivly.
t0 = 0;  tf = 1;
bounds.lower.time = [t0; tf];				
bounds.upper.time = bounds.lower.time;


%Set the box constraints for the endpoint constraint (i.e.: 'e' in the problem
%formulation).  These are the initial end end conditions for position and
%velocity.
x0 = 0; xf = 1;
v0 = 0; vf = 0;
bounds.lower.events = [x0; v0; xf; vf];
bounds.upper.events = bounds.lower.events;   %equality event function bounds



%Note that the path file not required for this problem formulation since the
%problem formulation does not have a path constraint.
linQ.cost 	  = 'costFun';
linQ.dynamics = 'dynamicsFun';
linQ.events	  = 'eventFun';		
linQ.bounds   = bounds;

%To make a guess, we guess the states control and time for the startTime and EndTime
if( makeGuess )
    ourGuess.states(1,:)=[x0, xf];
    ourGuess.states(2,:)=[v0, vf];
    
    ourGuess.controls(1,:) = [0,0];
    ourGuess.time = [t0, tf];
    
    algorithm.guess = ourGuess;
end

%We can use the primal structure as an initial guess for dido: Eg: Run 20 nodes with the
%made-up guess above and for a run with 32 nodes, we can use the results from the 20 node
%run as a guess to the optimal solution.
if( loadGuess )
    load linQuadPrimal primal;
    algorithm.guess = primal;
end

if( runAccruateMode )
    algorithm.mode = 'accurate';
end

%The number of nodes Nn is Nn+1 sample points taken inbetween the initial and
%final time bounds.  Increase Nn for greater accuracy.
algorithm.nodes = [Nn];


% call dido and record the execution time
tStart= cputime;
    [cost, primal, dual] = dido(linQ, algorithm);
runTime = cputime-tStart;

%%%%%%%%%%%%%%%%%%-----Pretty Plot Code Follows-----%%%%%%%%%%%%%%%%%%

disp(strcat('Dido executed in:', ' ' ,num2str(runTime), ' seconds.'));
disp(strcat('Minimized Cost:', ' ' ,num2str(cost), ' seconds.'));

%Unpack the optimal solution from DIDO's primal structure
xOpt = primal.states(1,:);
vOpt = primal.states(2,:);
uOpt = primal.controls;
tNodes = primal.nodes;  %time for this problem is [0,1]


%Plot the optimal control over time
plot( tNodes, uOpt);
title('Control (optimal) over time','fontSize',14,'fontWeight','bold');

%Here we plot the states x v and over time [t0,tf].
figure;
plot(tNodes, xOpt, '-o', tNodes, vOpt, '-x');
    title('Double Integrator States [x v]')
    xlabel('time');
    ylabel('states');
    legend('x', 'v');


%Plot the Control Hamiltonian.
figure;
plot(tNodes, dual.Hamiltonian);
    title('Double Integrator Hamiltonian Evolution');
    legend('H');
    xlabel('time');
    ylabel('Hamiltonian Value');


%Plot the costates lambda_x and lambda_v.
figure;
plot(tNodes, dual.dynamics);
    title('Double Integrator Costates')
    xlabel('time');
    ylabel('costates');
    legend('\lambda_x', '\lambda_v');

    
lambdaV = dual.dynamics(2,:);
lambdaX = dual.dynamics(1,:);
lowerHamil = -0.5*lambdaV.^2+ lambdaX.*vOpt;

figure;
plot(tNodes, lowerHamil);
    title('Lower Hamiltonian');

%{
    In this section, we perform Verification and Validation.  Using PMP we receive a
    collection of necessary conditions for optimality.  We compare the 
%}
%We found the extremal states x and v yet need to find a b c and d. We are solving, in the
%next few lines, the system: [ x(t0) v(t0 x(tf) v(tf)]'*[a b c d]' = [x0 v0 xf vf]'
T = @(tStart, tEnd) [ tStart^3/6  tStart^2/2  tStart  1;
                      tStart^2/2      tStart      1     0;
                      tEnd^3/6     tEnd^2/2    tEnd   1;
                      tEnd^2/2        tEnd        1     0 ];
%B is the column vector [x(t0) v(t0) x(tf) v(tf)]'
B = [x0; v0; xf; vf];
p = T(t0, tf)\B;

a = p(1);
b = p(2);
c = p(3);
d = p(4);

%The following was found via an application of the PMP (and above we found a b c and d :)
xExtremal = @(t) (a/6)*t.^3 + (b/2)*t.^2 + c*t + d;
vExtremal = @(t) (a/2)*t.^2 + b*t + c;
ctrlExtremal = @(t) a*t+b;
costateX = @(t) a;
costateV = @(t) -a*t-b;

%Next we compare the difference between the analyitical results from pen and paper to the
%ones received from DIDO.  Note that we are only checking necessary conditions, which is a
%Negative Test (!!!!) meaning that passing these doesn't mean we are garanteed the
%optimal.
errorXextremal = abs(xOpt - xExtremal(tNodes));
errorVextremal = abs(vOpt - vExtremal(tNodes));
errorCostateX = abs(lambdaX - costateX(tNodes));
errorCostateV = abs(lambdaV - costateV(tNodes));
errorCtrlExtremal = abs(uOpt - ctrlExtremal(tNodes));

%{
    Compare the difference between the analytical resuts from PMP and the numerical
    results from DIDO.
%}
figure;
plot( tNodes, errorXextremal, ...
      tNodes, errorVextremal, ...
      tNodes, errorCostateX, ...
      tNodes, errorCostateV, ...
      tNodes, errorCtrlExtremal );
title('Error between Analytic vs Numeric');
xlabel('t');
ylabel('error');
legend('xExtremal', 'vExtremal', 'costateX', 'costateV', 'controlExtremal');

%We save all of the variables used in the script to: 1. Use as an initial guess for the
%next run (which will have more nodes) 2. To perform feasibility Analysis
save linQuadPrimal;


%
% end linQuadProblemFile.m
%
