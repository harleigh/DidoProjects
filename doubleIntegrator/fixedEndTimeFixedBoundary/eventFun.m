%{
    eventFun: In this file we define e, the smooth terminal manifold to where
              the system must be transfered to by the final time.

    For the Linear Quadratic problem, the end condition is that the state [x v] at
    a spefified final time must be at a specific point x(tf)==xf and must be at rest, that
    is v(tf)=0, lastly in this problem the final time is exactly 1--i.e.: tf==1

    The event is set in the Problem File, so here we just need to pack the state
    variables at the final time in the order specified by bounds.event.lower and
    bounds.event.upper

    Inputs:
        primal: A structure array as given in M. Ross' indroduction to DIDO
                paper.  The fields relevent the the eventFun function are
                    x0 == primal.states(:,1) is a Nx by 1 column vector
                    xf == primal.states(:,end) is a Nx by 1 column vector
                    t0 == primal.nodes(1) is a 1 by 1 scalar
                    tf == primal.nodes(end) is a 1 by 1 scalar
    Outputs:
        endPointConstraints: A Ne by 1 numerical column vector that contains the
           boundary constraints to the Linear Quadratic problem.  The endpont
           constraints are ordered as [x0,v0,xf,yf] and this order is
           specified when setting 'bounds.lower.events' ( and similarly,
           setting 'bounds.upper.events') in the main file 'linQuadProblemFile'

           Note that the final time being set to 1 is set in the main file
           'linQuadProblemFile'
%}
function [ endPointConstraints ] = eventFun( primal )

    %Note the order of the states (that x is row 1 and v is row 2) is set by the box
    %constraints in the main file 'linQuadProblemFile'

    %pull out the initial and final value of the state variable x
    x0 = primal.states(1,1);
    xf = primal.states(1,end);
    
    %pull out the initial and final value of the state variable v
    v0 = primal.states(2,1);
    vf = primal.states(2,end);

    %The order of the endpoint constraints is [x0;v0;xf;vf], this has been set in the main
    %file.
    endPointConstraints = zeros(4,1);
    
    %We pack them (in the correct order)
    endPointConstraints(1) = x0;
    endPointConstraints(2) = v0;
    endPointConstraints(3) = xf;
    endPointConstraints(4) = vf;
    
end

