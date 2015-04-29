%{
    Tutorial on Dido: Showcasing the "Brach:1" problem formulation of the Brachistochrone
    problem, page 12 of Michael Ross' text A primer onPontryagin's Principle in Optimal
    Control.

 Problem Statement:
        A point mass is allowed to slide over riged two dimensional surface with
        (or without) a friction force acting oppisite to the velocity vector
        (assuming the friction law is given as \mu*N where \mu is the coeff of
        sliding friction) along with the influance of gravity: What is the shape
        of the surface that makes the point mass to move from one given point to
        another given point (i.e.: initial and final point) in Minimum Time?

    Formulation of the Brach Problem:
        Minimize   tf
        subject to the dynamics
             \dot x - v*sin(theta) = 0
             \dot y - v*cos(theta) = 0
             \dot v - g*cos(theta) = 0
        subject to the endpoint constraints
             t0  = 0
           x(t0) = x0   i.e.: the initial position in x
           y(t0) = y0   i.e.: the initial position in y
           v(t0) = v0   i.e.: the initial velocity of the point mass
           x(tf) = xf   i.e.: the final position of x
           y(tf) = yf   i.e.: the final position of y

    General Work-Flow of the main problem file
        * Define the box constraints on the State Variables
        * Define the box constraints on the Control Variables
        * Define initial and final time for the problem:  t0, tf
        * Define the endpoint constraints to the problem: e
        * Define the path constaint bounds  (Not used in this formulation)
        * Set up a guess to the solution of the problem
        * Set the number of nodes for the problem (16 is a good starting point)
%}
clear all; close all;

%Constant gravity
global g;

g = 9.8;

%These values are here because they are used so often.  Rather than scroll through the
%code, we can set these values conveinetly here.
Nn=16;
makeGuess = 0;
loadGuess = 0;
runAccurateMode = 0;

%Setting the box constraints to the OptCtrl problem.  Note that setting the box
%costraints determines the order of the state vectors.
%Careful when setting the box constraints: (1) Watch out for singularities in the box
%constraints (2) Watch out for setting them too large (3) Watch out for setting them too
%small.  The box constrains define the search space for the state variables.
xL = 0;    yL=0;    vL=0;
xU = 10;   yU=10;   vU=10;
bounds.lower.states = [xL; yL; vL];
bounds.upper.states = [xU; yU; vU];


%Set the box constraints for the control u.  Again, the box constrains define the search
%space for the control.
uL = 0; uU = pi;
bounds.lower.controls = [uL];
bounds.upper.controls = [uU];


%Set the bounds for the initial and final time.  Initial time starts at 0, but the final
%time, being free is set to a range [t0, tfMax].  Common issue is not setting tfMax large
%enough; if the bound for the final time is not large enough, Dido will claim the problem
%is infeasible.  (Many things can make Dido return an infeasibility statement, this is a
%common one that occurs)
t0 = 0;  tfMax = 10;
bounds.lower.time = [t0; t0];				
bounds.upper.time = [t0; tfMax];


%Set the the endpoint constraint (i.e.: 'e' in the problem
%formulation).  Here we start Brach at the origin and end at (xf,yf), giving an initial
%velocity of v0
x0 = 1; xf = 3;
y0 = 1; yf = 2;
v0 = 0;
bounds.lower.events = [x0; y0; v0; xf; yf];
bounds.upper.events = bounds.lower.events;   %equality event function bounds

%On a first run, giving a guess to what the solution might look like is very much 
%recomended.  The guess comes from insperation from the physics of the problem.  Dido can
%do its own guess, but giving your own is nearly always better (assuming it's good).
if( makeGuess )
    %Make a guess for all of the states
    ourGuess.states(1,:)	= [0, xf];   % state x
    ourGuess.states(2,:)	= [0, yf];   % state y
    ourGuess.states(3,:)	= [0, 0];    % state v
    
    %make a guess for all of your controls
    ourGuess.controls(1,:)		= [pi/4, pi/2];
    
    %We guess a time for each guess-entry to the states and controls; we guessed two for
    %for each, so we guess a time for each of them. For min final time problems, if your
    %guess for the final time is too small, the program will claim your problem is
    %infeasible.  Keep an eye out for this; quite common.
    ourGuess.time			    = [t0, tfMax];
    
    %The guess gets stored in the 'guess' field of the 'algorithim' structure
    algorithm.guess = ourGuess;
end

%On an initial run, it's good to provide a guess of your own making.  For future guesses,
%we can give, as our guess, the output of the previous run:  Here we take the results of
%the last run, and use it as a guess for the next run.
if( loadGuess )
	load optCtrlPrimal primal;
	algorithm.guess = primal;
end

%Only run accurate mode if you have had some success in non-accurate mode
if(runAccurateMode)
    algorithm.mode = 'accurate';
end

%The number of nodes Nn is Nn+1 sample points taken inbetween the initial and
%final time bounds.  Increase Nn for greater accuracy.  Hence Nn+1 points on the
%interval [t0,tf]
algorithm.nodes = [Nn];



%Here we pack the problem formulation for Dido to solve.
optCtrlProb.cost 	  = 'costFun';
optCtrlProb.dynamics  = 'dynamicsFun';
optCtrlProb.events	  = 'eventFun';		
optCtrlProb.bounds    = bounds;

% Call dido and record the execution time.  We pass in the problem formulation (containing
% the problem data) and the algorithim sturucture, which contains a guess (you should give
% a guess), the number of sample nodes (16 is a great starting value), and whether to run
% in accurate mode (only try this if you are succsesful on non-accurate mode)
tStart= cputime;
    [cost, primal, dual] = dido(optCtrlProb, algorithm);
runTime = cputime-tStart;

%%%%%%%%%%%%%%%%%%-----Pretty Plot Code Follows-----%%%%%%%%%%%%%%%%%%

disp(strcat('Dido executed in:', ' ' ,num2str(runTime), ' seconds.'));
disp(strcat('Cost Minimized as:', ' ' ,num2str(cost), ' timeUnits (seconds).'));

%Unpack the optimal solution from DIDO's primal structure
xOpt = primal.states(1,:);
yOpt = primal.states(2,:);
vOpt = primal.states(3,:);
uOpt = primal.controls;
tNodes = primal.nodes;

hold on;
    plot(xOpt,yOpt,'ro-');
    set(gca,'YDir','reverse'); grid on;
    title('Brachistochrone','fontSize',14,'fontWeight','bold');
    xlabel('x','fontSize',14,'fontWeight','bold');
    ylabel('y   ','fontSize',14,'fontWeight','bold');
    set(get(gca,'YLabel'),'Rotation',0) %flip the axis
hold off;


%Plot the optimal control over time
figure;
plot( tNodes, uOpt);
title('Control (optimal) over time','fontSize',14,'fontWeight','bold');


%Here we plot the states over time [t0,tf].
figure;
plot(tNodes, xOpt, '-o', tNodes, yOpt, '-x', tNodes, vOpt, '--');
    title('States')
    xlabel('time');
    ylabel('states');
    legend('x', 'y', 'v');


%Plot the Control Hamiltonian.  For this problem, it should be -1 for all time (given by
%the Hamiltonian Value Condition along with the Hammiltonian being time invariant).
figure;
plot(tNodes, dual.Hamiltonian);
    title('Hamiltonian');
    legend('H');
    xlabel('time');
    ylabel('Hamiltonian Value');


%Plot the costates
figure;
plot(tNodes, dual.dynamics);
    title('Costates')
    xlabel('time');
    ylabel('costates');
    legend('\lambda_x', '\lambda_y', '\lambda_v');



%We save all of the variables used in the script to: 1. Use as an initial guess for the
%next run  2. To be able to perform feasibility Analysis on the results returned by dido
save optCtrlPrimal;


%
% end mainProblemFile.m
%
