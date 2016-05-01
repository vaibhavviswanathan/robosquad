function [wallDist] =  getWallDistance(x, y, t, segment, mapSegmentCorners)

    x1 = mapSegmentCorners(segment, 1, 1);
    x2 = mapSegmentCorners(segment, 2, 1);

    y1 = mapSegmentCorners(segment, 1, 2);
    y2 = mapSegmentCorners(segment, 2, 2);

    if ((abs(x1 - x) >= 6.0 && abs(x2 - x) >= 6.0) || (abs(y1 - y) >= 6.0 && abs(y2 - y) >= 6.0))
        wallDist = 9999;
        return;
    end
    
%     minX = 9999; minY = 9999; maxX = -9999; maxY = -9999;
%     minX = min(minX, min(x1,x2));
%     minY = min(minY, min(y1,y2));
%     maxX = max(maxX, max(x1,x2));
%     maxY = max(maxY, max(y1,y2));
    
    m_wall = (mapSegmentCorners(segment,1,2)-mapSegmentCorners(segment,2,2))/(0.001+mapSegmentCorners(segment,1,1)-mapSegmentCorners(segment,2,1));
	b_wall = mapSegmentCorners(segment,1,2) - m_wall*mapSegmentCorners(segment,1,1);
    
    

    m_bot = tan(t);
    b_bot = y - m_bot * x;

    if(m_wall == m_bot)
        x_c =  100000000000;
    else
        x_c = (b_bot - b_wall) / (m_wall - m_bot);
    end
   
    y_c = m_wall * x_c + b_wall;

    tol = 0.001;

    validSeg = ((x_c >= min(x1, x2) - tol) && (x_c <= max(x1, x2) + tol)) && ((y_c >= min(y1, y2) - tol) && (y_c <= max(y1, y2) + tol));

    % check if in right heading of the robot
    validSeg = validSeg && (sign(t) == sign(y_c - y));

    if(validSeg)
        wallDist = sqrt((x_c - x)^2 + (y_c - y)^2);
    else
        wallDist = Inf;
    end
    %wallDist = validSeg ? (Pow((x_c - x), 2) + Pow((y_c - y), 2)) : Double.PositiveInfinity;


    % ****************** Additional Student Code: End   ************

end
