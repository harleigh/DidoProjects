%{
    This function is called to perform feasibility analysis to the Dubins Vehicle min
    energy problem.
%}
function XDots = prop_dynamic(tr,xr)

global  primal_temp V

%Find the (interpolated) value of Dido's optimal control at time tr
u   = interp1(primal_temp.nodes,primal_temp.controls(1,:),tr,'linear','extrap')';  

%Pull out the states from the vector
x = xr(1);
y = xr(2);
theta = xr(3);

%Dynamics of the Dubins Vehicle
xdot = V*cos(theta);
ydot = V*sin(theta);
thetadot = u;

%lastly we return the dynamics resultant from using the optimal control from Dido
XDots = [xdot; ydot; thetadot];
