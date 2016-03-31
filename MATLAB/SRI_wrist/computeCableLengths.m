function [L1,L2] = computeCableLengths(K,P)
    L1 = 0;
    for i = 2:1:size(K,1)-1
        L1 = L1 + norm(P(:,K(i-1,1))-P(:,K(i,1)));
       if K(i) == 4 
           break;
       end
    end
    L2 = 0;
    for i = size(K,1)-2:-1:1
        L2 = L2 + norm(P(:,K(i+1))-P(:,K(i)));
        if K(i) == 3 
           break;
        end
    end
end