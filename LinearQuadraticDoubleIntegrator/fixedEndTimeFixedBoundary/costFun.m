%{
    costFun: This file defines the cost function J[x(*),u(*),tf] to be minimized
                i.e.: J = E( X(tf), tf) + integrate(F(X(t),U(t),t))dt

    Inputs:
        primal: A structure array as given in M. Ross' Indroduction to DIDO
                paper.
                    primal.states is a Nx by Nn real matrix
                    x0 == primal.states(:,1) is a Nx by 1 column vector
                    xf == primal.states(:,end) is a Nx by 1 column vector
                    t0 == primal.nodes(1) is a 1 by 1 scalar
                    tf == primal.nodes(end) is a 1 by 1 scalar
                    
    Outputs:
        endPointCost: A 1 by 1 (scalar) that represents the end point cost
        runningCost:  A 1 by N numerical array of the running cost evaluated at
                      the discrete time points located in primal.nodes
  
                      Note that if there is no running cost, the variable must
                      be set to 0 (the scalar).
%}
function [ endPointCost, runningCost ] = costFun( primal )

    %No endpoint cost for the linQuad problem, hence set to the scalar zero.
    endPointCost = 0;
    
    %the control for the linQuad problem is 1 by Nn row vector, that is
    %[u(t0) ... u(tNn)] where t0==0 and tNn==1; that t0 and tf are set as 0 and
    %1 is in the problem file, at: bounds.time.lower and bounds.time.upper
    u=primal.controls;
    
    %The running cost for the linear Quadratic problem is just the quadratic of the
    %control.
    runningCost = 0.5*u.^2;

end

