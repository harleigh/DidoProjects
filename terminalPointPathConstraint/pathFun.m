function [ h ] = pathFun( primal )

    ux = primal.controls(1,:);
    uy = primal.controls(2,:);

    %The path constraint is 1 = ux^2+uy^2.  The equality constraint is set in the main
    %file; here we just compute the value.
    h = ux.^2 + uy.^2;
end

