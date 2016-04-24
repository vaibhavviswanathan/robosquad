clear
close all
clc

%%

X = [-1 -1 0 1 0;
     -1 1 0 1 2];
 
Cx = mean(X,2);

X = (X-repmat(Cx,1,5));

t = pi/6;
R = [cos(t), -sin(t);
    sin(t), cos(t)];

S = [1 1 1;1 1 1];

Y = R*X;

plot(X(1,:),X(2,:),'*--',Y(1,:),Y(2,:),'*--');

