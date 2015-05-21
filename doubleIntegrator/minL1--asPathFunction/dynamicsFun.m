%{
    dynamicsFun: provides the right hand side of the dynamics to the
                 Brachistochrone probmen--i.e.: provide f from the dynamics
                           x(t) = f(x(t),u(t),t)
                 The dynamics of the Brachistochrone problem is defined in
                 brachProblemFile.m
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
    
    uA = primal.controls(1,:);
    uB = primal.controls(2,:);

    
    XDot = [ v; uA-uB];  %vDot of the brach dynamics
end

