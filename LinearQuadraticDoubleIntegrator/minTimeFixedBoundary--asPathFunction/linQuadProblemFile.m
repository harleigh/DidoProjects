%{
    Implementation of a minimum time double integrator: Section 3.5 of Michael Ross' text:
    A primer on Pontryagin's Principle in Optimal Control

        State x v, where x is position, v is velocity
        Control u represents force, is bounded by -1 and 1 for all time
        Minimize transfer time
        Dynamics  dx = v and dv = u  (where d(.) is time derivative) Dynamics come from
            force=mass*acceleration
        Start at time zero: t0=0
        End time is free
        Object starts with a given position and velocity
        Object must end at position zero at rest
%}
clear all; close all;

Nn = 128;
makeGuess = 0;
loadGuess = 1;
runAccruateMode = 0;

%The Box constraints for the state (x,v)
xL = -20; xU = 20;
vL = -20; vU = 20;

%set the box constraints for the state
bounds.lower.states = [xL; vL];
bounds.upper.states = [xU; vU];


%set the box constraints for the control u. Note: u is unconstrained.
uL = -10; uU = 10;
bounds.lower.controls = [uL];
bounds.upper.controls = [uU];

%Set the constraints on the path function.  hL and hU represent the bounds to the control
%for all time.  We say that for all time hL<=u(t)<=hU
%Note: By setting the control constraints as a path function, we get access to the
%switching function (the time-varying Legrangian value from the Legrange Hamiltonian by
%the KKT conditions)
hL = -1; hU = 1;
bounds.lower.path = [hL];
bounds.upper.path = [hU]; 


%set the bounds for the initial and final time.  In this problem formulation, the final
%time is free; to do this we allow the final time to be in a range, the upper time (tfMax)
%should be large, but not too large; too large of a horizon could cause your problem to
%fail by falling into some odd local optima pit.
t0 = 0;  tfMax = 100;
bounds.lower.time = [t0; t0];				
bounds.upper.time = [t0; tfMax];

%Set the box constraints for the endpoint constraint (i.e.: 'e' in the problem
%formulation).
x0 = 5; xf = 0;
v0 = 1; vf = 0;
bounds.lower.events = [x0; v0; xf; vf];
bounds.upper.events = bounds.lower.events;   %equality event function bounds


%Note that the path file not required for this problem formulation since the
%problem formulation does not have a path constraint.
linQ.cost 	  = 'costFun';
linQ.dynamics = 'dynamicsFun';
linQ.events	  = 'eventFun';	
linQ.path     = 'pathFun';
linQ.bounds   = bounds;

%To make a guess, we guess the states control and time for the startTime and EndTime
if( makeGuess )
    ourGuess.states(1,:)=[x0, xf];
    ourGuess.states(2,:)=[v0, vf];
    
    ourGuess.controls(1,:) = [0,0];
    ourGuess.time = [t0, .1*tfMax];
    
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
tNodes = primal.nodes;

%Optimal Trajectory in the x-v plane
figure;
    plot(xOpt, vOpt);
    title('Phase Plane');
    xlabel('x');
    ylabel('v');

%Plot the control over time along with the switching variable; the Legrangian value from
%the Legrange Hamiltonian.  From the complementary conditions by KKT, we know the
%necessary switching structure the optimal control must have; we know this from the value
%of mu.
figure;
    plot( tNodes, uOpt, tNodes, dual.path);
    title('Control (optimal) over time','fontSize',14,'fontWeight','bold');
    legend('u', '\mu');
    
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
    title('Hamiltonian');
    legend('H');
    xlabel('time');
    ylabel('Hamiltonian Value');


%Plot the costates lambda_x and lambda_v.
figure;
plot(tNodes, dual.dynamics);
    title('Costates')
    xlabel('t');
    ylabel('costates');
    legend('\lambda_x', '\lambda_v');

%Save all variables from the current run with Nn time nodes on [t0,tf].  Note
%that the variable 'primal' can be used as an initial guess.
save linQuadPrimal;

%
% end linQuadProblemFile.m
%
