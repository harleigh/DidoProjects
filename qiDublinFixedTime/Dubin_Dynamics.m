%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dynamics of Dubin's vehicle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function XDots = Dubin_Dynamics(primal)

global V

%Pull out the states of the Dubins Vehicle from the primal data structure.  Note that time
%runs horizontally in Dido
x      = primal.states(1,:);		 
y      = primal.states(2,:);		
theta  = primal.states(3,:);		 

%Similarly time runs horizontally for the control as well.
w  = primal.controls(1,:);

%=======================================================
% Equations of Motion:
%=======================================================
xdot     = V*cos(theta);
ydot     = V*sin(theta);  
thetadot = w;							 
%-------------------------------------------------------

%Lastly we store the dynamics for Dido Nx by Nn matrix
XDots = [xdot; ydot; thetadot];
