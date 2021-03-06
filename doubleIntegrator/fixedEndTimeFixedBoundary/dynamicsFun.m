%{
    dynamicsFun: provides the right hand side of the dynamics to the
                 Linear Quadratic probmen--i.e.: provide f from the dynamics
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

    v = primal.states(2,:); %row vector of size Nx: v(t0)...v(tNn)
    u = primal.controls;    %Nu by Nn matrix (hence 1 by Nn for this problem)
    
    %Dynamics of the Linear Quadratic as given in the problem statement.
    XDot = [ v; u ];
end

