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
        XDot: A Nx by Nc numerical matrix which represents the right hand side
              of the dynamics--i.e.: f.  Note the capital 'x' of 'XDOT' is just
              to represent the state 'X'

%}
function [ XDot ] = dynamicsFun( primal )

    global v;
    
    theta = primal.states(3,:);
    omega = primal.controls;
    
    xDot = v*cos(theta);
    yDot = v*sin(theta);
    thetaDot = omega;
   
    %Dynamics go into XDot
    XDot = [ xDot; yDot; thetaDot ];
end

