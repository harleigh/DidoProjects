%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main file for path planning with Dubin's vehicle. Final time 
% is fixed. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;					

%{
  Global Variables:
    V:     A constant scalar (1x1) representing forward velocty
    primal_temp:  contains the data structure 'primal' returned by Dido.  This global is
                  only used in the feasibility analysis section; so we can interpolate the
                  optimal control returned by Dido.
%}
global V primal_temp

V  = 0.1;	% forward velocity

%-------------------------
% Define the problem:  Here 'Dubin' is our problem data
%-------------------------
Dubin.cost 	   = 'Dubin_CostU2';
Dubin.dynamics = 'Dubin_Dynamics';
Dubin.events   = 'Dubin_Events';

%------------------------------------------
% Set up the bounds and boundary conditions
%------------------------------------------

%In this problem, intial and final time are fixed
t0 = 0; tf = 20;

x0 = 0; y0 = 0; theta0 = pi/2;
xf = 1; yf = 0; thetaf = pi/2;

% setup the bounds on time
bounds.lower.time 	= [t0 tf];				
bounds.upper.time	= [t0 tf];		

% Bounds for the state variables.  Note that this specifies the order of the state
% varuables
bounds.lower.states = [-10; -10; -2*pi];
bounds.upper.states = [+10; +10; +2*pi];

% Bounds for omega the control
bounds.lower.controls = [-1000];
bounds.upper.controls = [+1000];

%Event Bounds, which are the start-points and end-points
bounds.lower.events = [x0; y0; theta0; xf; yf; thetaf];	
bounds.upper.events = [x0; y0; theta0; xf; yf; thetaf];   

%Store the bounds on the states/control/events into the 'bounds' field of the dido problem
%structure
Dubin.bounds = bounds;

%Setting the number of sample ponints for Dido
algorithm.nodes = [16];
        
%-----------------------------
% Provide a guess:  We must guess atleast two points for each state variable as well as
% each control, and we must guess a time for each of these events
%-----------------------------
guess.states(1,:)	= [x0, xf];
guess.states(2,:)	= [y0, yf];
guess.states(3,:)	= [theta0, thetaf];
guess.controls(1,:)	= [0, 0];
guess.time	    	= [t0, tf];


%We can also provide a guess to Dido by feading in the results of an old run.  For
%example, run 16 nodes with our guess, and then for a 32 node run, As a guess, give the
%results from the 16 node run
%------------------------
% load initial guess
%-----------------------
% load N16tf20V01 primal
% guess.states = primal.states;
% guess.controls = primal.controls;
% guess.time = primal.nodes;

algorithm.guess = guess;

[cost, primal, dual] = dido(Dubin, algorithm);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%          OUTPUT             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% feasibility verification
primal_temp = primal;
OPTIONS = odeset('AbsTol',1e-6, 'RelTol', 1e-6);
init = [x0;y0;theta0];
[t_prop,x_prop] = ode45('prop_dynamic', primal.nodes, init, OPTIONS);

%Here we plot the optimal states returned by Dido, as well as the results from above
%feasibility analysis
set(0,'defaulttextfontsize',16);
set(0,'DefaultAxesFontSize',16);
figure; plot(primal.nodes, primal.states, '*');
legend('x', 'y', 'theta');
hold on; grid on;
plot(t_prop,x_prop);
title('Path Planning -- Dubin Vehicle')
xlabel('normalized time units');
ylabel('Optimal States');


%Plot the control over time
figure
plot(primal.nodes, primal.controls,'*-'); grid on;
title('Path Planning -- Dubin Vehicle')
xlabel('normalized time units');
ylabel('Optimal Control');

%Plot the costates lambda_x lambda_y and lambda_thetaz
figure
plot(primal.nodes, dual.dynamics,'*-'); grid on; hold on;
title('Path Planning -- Dubin Vehicle')
xlabel('normalized time units');
ylabel('Costates');

%Plot the path of the Dubins Vehicle 
figure;
plot(primal.states(1,:),primal.states(2,:),'*-'); grid on; hold on;
title('Path Planning -- Dubin Vehicle')
xlabel('x'); ylabel('y');

% V&V using necessary conditions
HMC = 2*primal.controls+dual.dynamics(3,:);
figure;
plot(primal.nodes,HMC); grid on;
title('Path Planning -- Dubin Vehicle')
xlabel('normalized time units');
ylabel('Hamiltonian Minimization Condition');

%Plot the Control Hamiltonian
figure;
plot(primal.nodes,dual.Hamiltonian); grid on;
title('Path Planning -- Dubin Vehicle')
xlabel('normalized time units');
ylabel('Hamiltonian');

%%% save the result to the current folder
P = mfilename('fullpath');
[PATHSTR,NAME,EXT] = fileparts(P);
filename = fullfile(PATHSTR,'N16tf20V01.mat');
save(filename); 


