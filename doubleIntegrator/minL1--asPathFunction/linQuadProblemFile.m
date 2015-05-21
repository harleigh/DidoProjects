%{
    Implementation of a minimum time double integrator: Section 3.5 of Michael Ross' text:
    A primer on Pontryagin's Principle in Optimal Control

        State x v, where x is position, v is velocity
        Control u represents force, is bounded by -6 and 6 for all time
        Minimize transfer time
        Dynamics  dx = v and dv = u  (where d(.) is time derivative) Dynamics come from
            force=mass*acceleration
        Start and End time are Fixed
        Object starts with a given position and velocity
        Object must end at a given position at g velocity
%}
clear all; close all;

Nn = 128;
makeGuess = 0;
loadGuess = 1;
runAccruateMode = 0;

%The Box constraints for the state (x,v)
xL = -10; xU = 10;
vL = -10; vU = 10;

%set the box constraints for the state
bounds.lower.states = [xL; vL];
bounds.upper.states = [xU; vU];


%set the box constraints for the control u.  In this formulation we have two controls, uA
%and uB which allows us to smooth out the cost function
uAL = -10; uAU = 10;
uBL = -10; uBU = 10;
bounds.lower.controls = [uAL; uBL];
bounds.upper.controls = [uAU; uBU];

%Set the constraints on the path function.  hL and hU represent the bounds to the control
%for all time.  We say that for all time hL<=u(t)<=hU.  Note that uA>=0 and uB>=0 can also
%be set as box constraints.
hL = -6; hU = 6;
bounds.lower.path = [0;      0;  hL];
bounds.upper.path = [uAU;  uBU;  hU]; 


%set the bounds for the initial and final time.  In this problem formulation the initial
%and final times are fixed.
t0 = 0;  tf = 1;
bounds.lower.time = [t0; tf];				
bounds.upper.time = bounds.lower.time;

%Set the box constraints for the endpoint constraint (i.e.: 'e' in the problem
%formulation).
x0 = 0; xf = 1;
v0 = 0; vf = 0;
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
    ourGuess.states(1,:)=[x0, xf];      %Guess for state x
    ourGuess.states(2,:)=[v0, vf];      %Guess for state v
    
    ourGuess.controls(1,:) = [0, 6];    %Guess for uA
    ourGuess.controls(2,:) = [0, 6];    %guess for uB
    
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
mu = dual.path;
tNodes = primal.nodes;

%Optimal Trajectory in the x-v plane
figure;
    plot(xOpt, vOpt);
    title('Phase Plane');
    xlabel('x');
    ylabel('v');
    grid on;

%Plot the control over time along with the switching variable;
figure;
    plot( tNodes, (1/6)*(uOpt(1,:)-uOpt(2,:)), tNodes, mu);
    title('Control (optimal) over time','fontSize',14,'fontWeight','bold');
    legend('u', '\mu_1', '\mu_2', '\mu_3');
    grid on;
    
%Here we plot the states x v and over time [t0,tf].
figure;
plot(tNodes, xOpt, '-o', tNodes, vOpt, '-x');
    title('Double Integrator States [x v]')
    xlabel('time');
    ylabel('states');
    legend('x', 'v');
    grid on;


%Plot the Control Hamiltonian.
figure;
plot(tNodes, dual.Hamiltonian);
    title('Hamiltonian');
    legend('H');
    xlabel('time');
    ylabel('Hamiltonian Value');
    grid on;


%Plot the costates lambda_x and lambda_v.
figure;
plot(tNodes, dual.dynamics);
    title('Costates')
    xlabel('t');
    ylabel('costates');
    legend('\lambda_x', '\lambda_v');
    grid on;

%Save all variables from the current run with Nn time nodes on [t0,tf].  Note
%that the variable 'primal' can be used as an initial guess.
save linQuadPrimal;

%
% end linQuadProblemFile.m
%
