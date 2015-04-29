%{
    dynamicsFun: provides the right hand side of the dynamics to the
                 optimal control probmen--i.e.: provide f from the dynamics
                           x(t) = f(x(t),u(t),t)
    Inputs:
        primal: A structure array as given in M. Ross' indroduction to DIDO
                paper.
                    primal.states is a Nx by Nn real matrix whose iTh row and
                    jTh column is x_i(t_j)
    Outputs:
        XDot: A Nx by Nn numerical matrix which represents the right hand side
              of the dynamics--i.e.: f.  Note the capital 'x' of 'XDot' is just
              to represent the state vector 'X'

%}
function [ XDot ] = dynamicsFun( primal )

    global g;

    %The order of the states is given by setting the box constraints in the main file
    v = primal.states(3,:);
    
    %In this brach formulation, control is just a 1 by Nn array
    theta = primal.controls;
    
    
    xDot = v.*sin(theta);
    yDot = v.*cos(theta);
    vDot = g*cos(theta);
    
    
    %Dynamics go into XDot, which is a Nx by Nn matrix
    XDot = [ xDot; yDot; vDot ];
end

%
% end dynamicsFun.m
%
