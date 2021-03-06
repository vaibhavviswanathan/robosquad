%% Import data from text file.
% Script for importing data from the following text file:
%
%    /Users/kunalmenda/Google Drive/2016 Spring/E160/robosquad/Jaguar_BaseCode_05/bin/x86/Release/JaguarData_2016-4-18-33.txt
%
% To extend the code to different selected data or a different text file,
% generate a function instead of a script.

% Auto-generated by MATLAB on 2016/04/18 19:43:42

%% Initialize variables.
filename = '/Users/kunalmenda/Google Drive/2016 Spring/E160/robosquad/Jaguar_BaseCode_05/bin/x86/Release/JaguarData_2016-4-18-49.txt';
delimiter = ' ';

%% Format string for each line of text:
%   column1: double (%f)
%	column2: double (%f)
%   column3: double (%f)
%	column4: double (%f)
%   column5: double (%f)
%	column6: double (%f)
%   column7: double (%f)
%	column8: double (%f)
%   column9: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'EmptyValue' ,NaN, 'ReturnOnError', false);

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Allocate imported array to column variable names
time = dataArray{:, 1};
x = dataArray{:, 2};
y = dataArray{:, 3};
t = dataArray{:, 4};
xkf = dataArray{:, 5};
ykf = dataArray{:, 6};
tkf = dataArray{:, 7};
sxx = dataArray{:, 8};
syy = dataArray{:, 9};
sxy = dataArray{:, 10};
inds = 1:10:length(t);
time = time(inds);
x = x(inds);
y = y(inds);
t = t(inds);
xkf = xkf(inds);
ykf = ykf(inds);
tkf = tkf(inds);
sxx = sxx(inds);
syy = syy(inds);
sxy = sxy(inds);


%% Clear temporary variables
clearvars filename delimiter formatSpec fileID dataArray ans inds;