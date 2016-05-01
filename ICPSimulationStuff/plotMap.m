function [ h ] = plotMap(  )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

mapSegCorners;

C = [         0    0.4470    0.7410];

h = figure();
hold on;

for i = 1:length(mapSegmentCorners)
    x1 = mapSegmentCorners(i,1,1);
    x2 = mapSegmentCorners(i,2,1);
    y1 = mapSegmentCorners(i,1,2);
    y2 = mapSegmentCorners(i,2,2);
    plot([x1,x2],[y1,y2],'Color',C);
    
end


end

