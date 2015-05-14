%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cost function for path planning with Dubin's vehicle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [endpointCost, runningCost] = Dubin_CostU2(primal)

%Time runs horizontally for the control as well.
w  = primal.controls(1,:);	    %row vector 1xNn

%No endpoint const for this problem formulation, so we must set it to zero
endpointCost = 0;
runningCost = 1*w.*w;

