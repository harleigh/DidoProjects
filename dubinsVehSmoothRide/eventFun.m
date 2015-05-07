%{
    eventFun: In this file we define e, the smooth terminal manifold to where
              the system must be transfered to by the final time.

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
           boundary constraints to the Opt Ctrl problem.

        Note that time bounds are set in the main file
%}
function [ endPointConstraints ] = eventFun( primal )

    %pull out the initial and final value of the state variable x
    initialStates = primal.states(:,1);
    finalStates = primal.states(:,end);
    
    %The order of the endpoint constraints is [x0;v0;xf;vf], this has been set in the main
    %file.
    endPointConstraints = [initialStates; finalStates];
    

end

