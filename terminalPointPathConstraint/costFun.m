%{
    costFun: This file defines the cost function to be minimized
                i.e.: J = E( X(t0), X(tf), t0, tf) + integrate(F(X(t),U(t),t))dt

             The cost function to be minimized for the Brachistochrone problem
             as defined in brachProblemFile.m

             The only cost to the brach problem is the endpoint cost (final
             time).  Hence the running cost is set to zero.

    Inputs:
        primal: A structure array as given in M. Ross' indroduction to DIDO
                paper.
                    primal.states is a Nx by Nn real matrix
                    x0 == primal.states(:,1) is a Nx by 1 column vector
                    xf == primal.states(:,end) is a Nx by 1 column vector
                    t0 == primal.nodes(1) is a 1 by 1 scalar
                    tf == primal.nodes(end) is a 1 by 1 scalar
                    
    Outputs:
        endPointCost: A 1 by 1 (scalar) that represents the end point cost
        runningCost: A 1 by N numerical array of the running cost evaluated at
                     the discrete time points located in primal.nodes  Note that
                     if there is no running cost, the variable must be set to 0
                     (the scalar).
%}
function [ endPointCost, runningCost ] = costFun( primal )

    tf = primal.nodes(end); %location of final time in the primal structure
    endPointCost = tf;      %end point cost is just the final time.
    runningCost = 0;        %no running cost so set to (scalar) zero

end

