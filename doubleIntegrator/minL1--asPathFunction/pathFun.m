function [ h ] = pathFun( primal )
    
    uA = primal.controls(1,:);
    uB = primal.controls(2,:);


    h(1,:) = uA;
    h(2,:) = uB;
    h(3,:) = uA-uB;

end

