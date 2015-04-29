%{
    What are we doing?  Solving the Brachistochrone proble, using the formulation
    'Brach:2' in section 1.1.5 in M. Ross' text.

    This formulation showcases path constraints, and a control of more than one dimension.
%}
clear all; close all;

%To share constants over the methods within dido, we use globals.
global g;

%gravity near sea level
g=9.8;


Nn=16;
makeGuess=1;
loadGuess=0;

%The Box constraints for the state (x,y,v)
xL = 0; xU = 20;
yL = 0; yU = 20;
vL = 0; vU = 20;

%set the box constraints for the state
bounds.lower.states = [xL; yL; vL];
bounds.upper.states = [xU; yU; vU];


%set the box constraint(s) for the control
uxL = -2; uxU = 2;
uyL = -2; uyU = 2;
bounds.lower.controls = [uxL; uyL];
bounds.upper.controls = [uxU; uyU;];


%set the bounds for the initial and final time
t0 = 0;
tfMax = 10; %guess for final time of the Brach; if this is too small Dido might say the problem is infeasible
bounds.lower.time = [t0; t0];				
bounds.upper.time = [t0; tfMax];


%Set the box constraints for the endpoint constraint (i.e.: 'e' in the problem
%formulation).  We want the particle mass to begin at (x0,y0) and to end at the
%point (xf,yf) and start with initial velocity of v0.
x0 = 1; y0 = 1; v0 = 0;
xf = 4;  yf = 2;
bounds.lower.events = [x0; y0; v0; xf; yf];
bounds.upper.events = bounds.lower.events;   %equality event function bounds

%Path constraints ux^2+uy^2=1.  Here we set that the path must be equal to 1.
bounds.lower.path = [1];
bounds.upper.path = [1];

if( makeGuess )
    %Make a guess for all of the states
    ourGuess.states(1,:)	= [0, xf];   % state x
    ourGuess.states(2,:)	= [0, yf];   % state y
    ourGuess.states(3,:)	= [0, 3];    % state v
    
    %make a guess for all of your controls
    ourGuess.controls(1,:)		= [0, pi/2];
    ourGuess.controls(2,:)		= [0, pi/2];
    
    %We guess a time for each guess-entry to the states and controls; we guessed two for
    %for each, so we guess a time for each of them. For min final time problems, if your
    %guess for the final time is too small, the program will claim your problem is
    %infeasible.  Keep an eye out for this; quite common.
    ourGuess.time			    = [t0, 3];
    
    %The guess gets stored in the 'guess' field of the 'algorithim' structure
    algorithm.guess = ourGuess;
end

if( loadGuess )
    load terminalPointPrimal primal;
    algorithm.guess = primal;
end

%The number of nodes Nn is Nn+1 sample points taken inbetween the initial and
%final time bounds.  Increase Nn for greater accuracy.
algorithm.nodes = [Nn];

%Note that the path file not required for this problem formulation since the
%problem formulation does not have a path constraint.
brachStruct.cost 	 = 'costFun';
brachStruct.dynamics = 'dynamicsFun';
brachStruct.events	 = 'eventFun';		
brachStruct.path     = 'pathFun';
brachStruct.bounds   = bounds;

% call dido and record the execution time
tStart= cputime;
    [cost, primal, dual] = dido(brachStruct, algorithm);
runTime = cputime-tStart;

%%%%%%%%%%%%%%%%%%-----Pretty Plot Code Follows-----%%%%%%%%%%%%%%%%%%

disp(strcat('Dido executed in:', ' ' ,num2str(runTime), ' seconds.'));
disp(strcat('Minimum Time is:', ' ' ,num2str(cost), ' seconds.'));

%Unpack the optimal solution from DIDO's primal structure
xOpt = primal.states(1,:);
yOpt = primal.states(2,:);
vOpt = primal.states(3,:);
uOpt = primal.controls;
tNodes = primal.nodes;


%Plot the brachistochrone curve in the xy plane (and flip the y-axix as
%yPositive was defined as going down durring the problem formulation).
hold on;
    plot(xOpt,yOpt,'ro-');
    set(gca,'YDir','reverse'); grid on;
    title('Brachistochrone','fontSize',14,'fontWeight','bold');
    xlabel('x','fontSize',14,'fontWeight','bold');
    ylabel('y   ','fontSize',14,'fontWeight','bold');
    set(get(gca,'YLabel'),'Rotation',0) %flip the axis
hold off;

%Plot the control over time.  Note that in some applications, the control could
%be quite nonSmooth, and thus when performing feasibility, one must be carefull
%when interpolating over such a nonSmooth curve.
figure; plot( tNodes, uOpt);
title('Control (optimal) over time','fontSize',14,'fontWeight','bold');

%Here we plot the states x y and vover time.  At tf we want the xComponent of
%the state to be at xf and similarly the yComponent to be at yf.
figure;
plot(tNodes, xOpt, '-o', tNodes, yOpt, '-x', tNodes, vOpt, '-.');
    title('Brachistochrone States [x y v]')
    xlabel('time');
    ylabel('states');
    legend('x', 'y', 'v');


%Plot the Hamiltonian.  By the Hamiltonian value condition, and that this
%problem is time invariant, we have (by theory) that the (control) hamiltonial
%as -1 for all time.
figure;
plot(tNodes, dual.Hamiltonian);
    title('Brachistochrone Hamiltonian Evolution');
    legend('H');
    xlabel('time');
    ylabel('Hamiltonian Value');


%Plot the costates lambda_x lambda_y and lambda_v.  By theory, we know that both
%lambda_x and lambda_y should be constants (from the Terminal Transversality
%Condition), whereas lambda_v should be zero at the final time (also by the TTC)
figure;
plot(tNodes, dual.dynamics);
    title('Brachistochrone Costates: Brac 1')
    xlabel('time');
    ylabel('costates');
    legend('\lambda_x', '\lambda_y', '\lambda_v');

%Save all the variables of the current run.  Note that variable 'primal' can be
%used as an initial guess to a new run (with more time nodes)
save terminalPointPrimal;
    
%
% end brachProblemFile.m
%
