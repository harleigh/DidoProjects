%{
    Using Dido, we solve the brachistochrone problem where we must end on a circle.
%}
clear all; close all;

%{
    g: Gravit Constant , Units m/s^2
    e: Function representing the circle the particle must land upon at the final time.
%}
global g e;


g = 9.8;
h = 5; k = 1; r =1/sqrt(10);
e = @(xf, yf) (xf-h)^2 + (yf-k)^2 - r^2;


Nn = 32;
makeGuess = 0;
loadGuess = 1;

%Setting the box constraints to the states
xL = 0; xU = 20;
yL = 0; yU = 20;
vL = 0; vU = 20;
bounds.lower.states = [xL; yL; vL];
bounds.upper.states = [xU; yU; vU];


%set the box constraints for the control
thetaL = 0; thetaU = pi;
bounds.lower.controls = [thetaL];
bounds.upper.controls = [thetaU];


%set the bounds for the initial and final time.  The final time is free, and the initial
%time is fixed
t0 = 0;  tfMax = 10;
bounds.lower.time = [t0; t0];				
bounds.upper.time = [t0; tfMax];


%Endpoint constraints: We are given an initial condition, and must end on a circle.  The
%last constraint defines e(x(tf),y(tf))=0
x0 = 0; y0=0; v0=0;
bounds.lower.events = [x0; y0; v0; 0];
bounds.upper.events = bounds.lower.events;


%Pack the problem data into the problem structure 'optCtrlProb'
optCtrlProb.cost 	   = 'costFun';
optCtrlProb.dynamics   = 'dynamicsFun';
optCtrlProb.events     = 'eventFun';		
optCtrlProb.bounds     = bounds;

%To use the previous run as an initial guess for the new run
if( loadGuess )
    load optCtrlPrimal primal;
    algorithm.guess = primal;
end

%A guesss to the problem: useful on a cold-run.  Our guess is a straight line to the
%center of the circle we must land upon.
if( makeGuess ) 
    guess.states(1,:)	= [x0, h];
    guess.states(2,:)	= [y0, k];
    guess.states(3,:)	= [v0, vU];
    
    guess.controls(1,:)		= [0, 0];
    
    guess.time			    = [t0, tfMax];
    
    algorithm.guess = guess;
end

%Here is how to run DIDO in accurate mode.
% algorithm.mode = 'accurate';

%The number of nodes Nn is Nn+1 sample points taken inbetween the initial and
%final time bounds.  Increase Nn for greater accuracy.  Hence Nn+1 points on the
%interval [0,1]
algorithm.nodes = [Nn];

% call dido and record the execution time
tStart= cputime;
    [cost, primal, dual] = dido(optCtrlProb, algorithm);
runTime = cputime-tStart;

%%%%%%%%%%%%%%%%%%-----Pretty Plot Code Follows-----%%%%%%%%%%%%%%%%%%

disp(strcat('Dido executed in:', ' ' ,num2str(runTime), ' seconds.'));
disp(strcat('Minimized Cost:', ' ' ,num2str(cost), ' cost units.'));

%Unpack the optimal solution from DIDO's primal structure
xOpt = primal.states;
uOpt = primal.controls;
tNodes = primal.nodes;


%%% Pretty Plot Code %%%

%Plot the Brach curve, as well as the terminal circle
figure;
hold on;
    tDom = 0:0.01:2*pi;
    plot(h+r*cos(tDom), k+r*sin(tDom));
    plot(xOpt(1,:),xOpt(2,:),'ro-');
    title('Brachistochrone with a circle Target Set','fontSize',20);
    xlabel('x','fontSize',20);
    ylabel('y   ','fontSize',20, 'Rotation', 0);
    set(gca,'YDir','reverse');
    grid on;
    axis equal;
hold off;

figure;
    plot(tNodes, xOpt);
    title('States', 'fontSize',20);
    xlabel('t','fontSize',20);
    grid on;
    legend('x','y','v');
    
figure
    plot(tNodes, uOpt);
    title('Control', 'fontSize',20);
    grid on;
    xlabel('t','fontSize',20);
    
figure;
    plot(tNodes, dual.Hamiltonian);
    title('Hamiltonian', 'fontSize',20);
    grid on;
    xlabel('t','fontSize',20);
    
figure;
    plot(tNodes, dual.dynamics);
    title('Costates', 'fontSize',20);
    xlabel('t','fontSize',20);
    grid on;
    legend('\lambda_x', '\lambda_y', '\lambda_v');
    
save optCtrlPrimal;

%
% end mainProblemFile.m
%
