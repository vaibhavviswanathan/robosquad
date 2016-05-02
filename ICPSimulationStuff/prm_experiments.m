close all

%Create map
plotMap
hold all;


% loop through all unoptimized files
unoptimized = strcat(pwd, '/unoptimized/*.txt');
files = dir(unoptimized)

for file = dir(unoptimized)'
    figure
    
    %Create map
    plotMap;
    hold all;
   %% Load file into MATLAB
   %% Load file into MATLAB
    file_name = file.name;
    %% Initialize variables.
    filename = strcat('C:\Users\Vaibhav\Documents\GitHub\robosquad\ICPSimulationStuff\unoptimized\', file_name);
    delimiter = ', ';

    %% Format string for each line of text:
    %   column1: double (%f)
    %	column2: double (%f)
    %   column3: double (%f)
    %	column4: double (%f)
    % For more information, see the TEXTSCAN documentation.
    formatSpec = '%f%f%f%f%[^\n\r]';

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


%% Clear temporary variables
clearvars filename delimiter formatSpec fileID dataArray ans;
   
   %compute length of path
   x_dist = diff(x);
   y_dist = diff(y);
   dist = sum((x_dist.^2 + y_dist.^2).^0.5)
   
   %plot path
   plot(x,y, 'r.')
   xlabel('x (meters)');
   ylabel('y (meters)');
end
