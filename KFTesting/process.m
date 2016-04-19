clear
close all
clc


loaddata

%%

figure();
plot(x,y);
hold on
plot(xkf,ykf);
for i = 1:10:length(sxx)
    plotConfidenceEllipse(sxx(i),syy(i),sxy(i),xkf(i), ykf(i));
end