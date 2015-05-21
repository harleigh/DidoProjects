%{
    eventFun: In this file we define e, the smooth terminal manifold to where
              the system must be transfered to by the final time.

    Inputs:
        primal: A structure array as given in M. Ross' indroduction to DIDO
                paper.  The fields relevent the the eventFun function are
                    x0 == primal.states(:,1) is a Nx by 1 column vector
                    xf == primal.states(:,end) is a Nx by 1 column vector
                    t0 == primal.nodes(1) is a 1 by 1 scalar
                    tf == primal.nodes(end) is a 1 by 1 scalar
    Outputs:
        endPointConstraints: A Ne by 1 numerical column vector that contains the
           boundary constraints to the Brachistochrone problem.  The endpont
           constraints are ordered as [x0,y0,v0,xf,yf] and the order is
           specified whenn setting 'bounds.lower.events' ( and similarly,
           setting 'bounds.upper.events')

(note: in 'Imputs' above,
           x0 and xf represent the state (column) vectors [x0 y0 v0] and [xf yf
           vf] since we can't do bold comments.
%}
function [ endPointConstraints ] = eventFun( primal )

    x0 = primal.states(1,1);
    xf = primal.states(1,end);
    
    v0 = primal.states(2,1);
    vf = primal.states(2,end);
    
    endPointConstraints = zeros(4,1);
    
    endPointConstraints(1) = x0;
    endPointConstraints(2) = v0;
    endPointConstraints(3) = xf;
    endPointConstraints(4) = vf;
end

