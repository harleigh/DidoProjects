%{
    eventFun: In this file we define e, the smooth terminal manifold to where
              the system must be transfered to by the final time.

    The event is set in the Problem File, so here we just need to pack the state
    variables in the order specified in the main problem file bounds.events.lower and
    bounds.events.upper

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
    x0 = primal.states(1,1);
    xf = primal.states(1,end);
    
    %pull out the initial and final value of the state variable v
    y0 = primal.states(2,1);
    yf = primal.states(2,end);
    
    %pull out the initial value of the state variable v
    v0 = primal.states(3,1);


    %The order of the endpoint constraints is set in the main file.
    endPointConstraints = zeros(5,1);
    
    %We pack them (in the order given in the main problem file)
    endPointConstraints(1) = x0;
    endPointConstraints(2) = y0;
    endPointConstraints(3) = v0;
    endPointConstraints(4) = xf;
    endPointConstraints(5) = yf;
    
end

%
% end eventFun.m
%
