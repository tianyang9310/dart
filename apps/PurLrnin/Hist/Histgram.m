clc,clear;
close all;

droppath = '/Users/Yang/Dropbox/';
datapath = [droppath 'Print-Archive/data/0307/'];

edges{1} = -0.5:1:9.5;
edges{2} = -0.5:1:9.5;

%% 
numContactToLearn = 8;
disp([datapath 'lcp_data' num2str(numContactToLearn) '.csv'])
data = csvread([datapath 'lcp_data' num2str(numContactToLearn) '.csv']);
for i = 1: numContactToLearn
    figure(i);
    hist(data(:,end-i+1));
    title(['contact ' num2str(i)]);
end


figure(numContactToLearn+1)
hist3(data(:,end-2:end-1),'EDGES',edges);