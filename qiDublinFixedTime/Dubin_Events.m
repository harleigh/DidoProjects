%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Event function for path planning with Dubin's vehicle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function e = Dubin_Events(primal)

%Pull the initial and final of each state from the primal data structure
x0     = primal.states(1,1);
y0     = primal.states(2,1);
theta0 = primal.states(3,1);  
xf     = primal.states(1,end);
yf     = primal.states(2,end);
thetaf = primal.states(3,end);  		

e = zeros(6,1);

%The packing order of e is dermined by the order set from bounds.lower.events and
%bounds.upper.events in the main problem file
%===========================================================
e(1) = x0; 
e(2) = y0; 
e(3) = theta0;
e(4) = xf; 
e(5) = yf; 
e(6) = thetaf;
%-----------------------------------------------------------

