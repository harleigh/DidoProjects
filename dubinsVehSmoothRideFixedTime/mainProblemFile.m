%{
    Here we look at a Dubins Vehicle: We control the turning rate and minimize the L2 norm
    of the turning rate (smooth ride)
    State: x y theta
    Control: omega (turning rate)
    Dynamics:
         xDot = v*cos(theta)
         yDot = v*sin(theta)
         thetaDot = omega
    So, we have a 3dim state and the control is one dim.

    We are given the initial position and orientation of the car, and we are given the
    final position and and orientation.  We are also given the start and end times.  Given
    this information we look for a curve that satisfies the endpoint constraints (initial
    and final) while minimizing the rate of change of the car turning.
%}
clear all; close all;

%velocity v is a constant
global v;

v = 0.1;

Nn = 16;
makeGuess = 1;
loadGuess = 0;
runInAccurateMode = 0;

%Setting the box constraints to the OptCtrl problem.
xL=-10;  yL=-10;  thetaL=-2*pi;
xU=10;   yU=10;   thetaU=2*pi;
bounds.lower.states = [xL;yL;thetaL];
bounds.upper.states = [xU;yU;thetaU];


%Set the box constraints for the control u.
omegaL = -100;
omegaU = 100;
bounds.lower.controls = [omegaL];
bounds.upper.controls = [omegaU];


%set the bounds for the initial and final time.
t0 = 0;  tf = 50;
bounds.lower.time = [t0; tf];				
bounds.upper.time = [t0; tf];


%Endpoint constraint (i.e.: 'e' in the problem formulation).  We start and end at a
%specific point and orientation
x0 = 0; y0 = 0;
xf = 1; yf = 1;
theta0 = pi/4;
thetaf = pi/4;
bounds.lower.events = [x0; y0; theta0; xf; yf; thetaf];
bounds.upper.events = bounds.lower.events;


dubinsVeh.cost 	   = 'costFun';
dubinsVeh.dynamics = 'dynamicsFun';
dubinsVeh.events   = 'eventFun';
dubinsVeh.bounds   = bounds;

%To use the previous run as an initial guess for the new run
if( loadGuess )
    load optCtrlPrimal primal;
    algorithm.guess = primal;
end

%A guesss to the problem: useful on a cold-run.  Our guess gives initial and final values
%for each state: x: 0 -> L, y:0->0, z:0->L
if( makeGuess ) 
    guess.states(1,:)	= [x0, xf];
    guess.states(2,:)	= [y0, yf];
    guess.states(3,:)	= [theta0, thetaf];
    
    guess.controls(1,:)		= [0, 0];
    
    guess.time			    = [t0, tf];
    
    algorithm.guess = guess;
end

if( runInAccurateMode )
    algorithm.mode = 'accurate';
end

%The number of nodes Nn is Nn+1 sample points taken inbetween the initial and
%final time bounds.  Increase Nn for greater accuracy.  Hence Nn+1 points on the
%interval [0,1]
algorithm.nodes = [Nn];

% call dido and record the execution time
tStart= cputime;
    [cost, primal, dual] = dido(dubinsVeh, algorithm);
runTime = cputime-tStart;

%%%%%%%%%%%%%%%%%%-----Pretty Plot Code Follows-----%%%%%%%%%%%%%%%%%%

disp(strcat('Dido executed in:', ' ' ,num2str(runTime), ' seconds.'));
disp(strcat('Cost Minimized:', ' ' ,num2str(cost), ' (cost units).'));

%Unpack the optimal solution from DIDO's primal structure
xOpt = primal.states;
omegaOpt = primal.controls;
tNodes = primal.nodes;

figure;
    plot(xOpt(1,:), xOpt(2,:));
    title( 'Curve Minimizing Engergy', 'FontSize', 20 );
    xlabel('x', 'FontSize', 20);
    ylabel('y', 'FontSize', 20);
    axis equal;

figure;
    plot(tNodes, xOpt);
    title('States', 'FontSize', 20);
    legend('x', 'y', '\theta');
    xlabel('t');

figure;
    plot(tNodes, omegaOpt)
    title('Control', 'FontSize', 20);
    xlabel('t');
    ylabel('u');
    
%Plot the Control Hamiltonian.
figure;
plot(tNodes, dual.Hamiltonian);
    title('Hamiltonian', 'FontSize', 20);
    legend('H');
    xlabel('time', 'FontSize', 20);
    ylabel('Hamiltonian Value', 'FontSize', 20);


%Plot the costates
figure;
plot(tNodes, dual.dynamics);
    title('Costates', 'FontSize', 20)
    xlabel('time', 'FontSize', 20);
    ylabel('costates', 'FontSize', 20);
    legend('\lambda_x', '\lambda_y', '\lambda_\theta');
    
%{
    Verification and Validation
%}
lambdaTheta = dual.dynamics(3,:);
%The HMC should be zero for all time
hamilMinCondition = 2*omegaOpt + lambdaTheta;
figure;
    plot(tNodes, hamilMinCondition);
    title('HMC', 'FontSize', 20);
    xlabel('t', 'FontSize', 20);
    ylabel('$\frac{\partial H}{\partial \omega}$', 'FontSize', 20, 'Rotation', 0, 'Interpreter','latex');

save optCtrlPrimal;

%
% end mainProblemFile.m
%
