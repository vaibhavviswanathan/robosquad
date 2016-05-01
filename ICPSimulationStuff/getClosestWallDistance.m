function [minDist] = getClosestWallDistance(x,y,t)
    
    mapSegCorners;
    minDist = Inf;
    for i = 1:length(mapSegmentCorners)
        dist = getWallDistance(x,y,t,i,mapSegmentCorners);
        minDist = min(dist,minDist);
    end

end