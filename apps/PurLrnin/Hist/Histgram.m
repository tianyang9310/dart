clc,clear;
close all;

path='/Users/Yang/Material/Research/dart/build/data/'
edges{1} = -0.5:1:9.5
edges{2} = -0.5:1:9.5

figure(1);
lcpdata4=csvread([path 'lcp_data4.csv']);
subplot(2,2,1)
hist(lcpdata4(:,end-3))
title('contact 1')
subplot(2,2,2)
hist(lcpdata4(:,end-2))
title('contact 2')
subplot(2,2,3)
hist(lcpdata4(:,end-1))
title('contact 3')
subplot(2,2,4)
hist(lcpdata4(:,end))
title('contact 4')

%% 
% clc,clear;
% close all;
figure(2);
% lcpdata3=csvread([path 'lcp_data3.csv']);
% lcpdata3=csvread([path 'lcp_data3_diag89.csv']);
lcpdata3=csvread([path 'lcp_data3_diag_no89.csv']);
% lcpdata3=csvread([path 'new_lcp_data3.csv']);
subplot(2,2,1)
hist(lcpdata3(:,end-2))
title('contact 1')
subplot(2,2,2)
hist(lcpdata3(:,end-1))
title('contact 2')
subplot(2,2,3)
hist(lcpdata3(:,end))
title('contact 3')

figure(6)
subplot(3,1,1)
hist3(lcpdata3(:,end-2:end-1),'EDGES',edges);
subplot(3,1,2)
hist3(lcpdata3(:,end-1:end),'EDGES',edges);
subplot(3,1,3)
hist3(lcpdata3(:,end-2:2:end),'EDGES',edges);

figure(3);
lcpdata2=csvread([path 'lcp_data2.csv']);
subplot(2,1,1)
hist(lcpdata2(:,end-1))
title('contact 1')
subplot(2,1,2)
hist(lcpdata2(:,end))
title('contact 2')
% hist3(lcpdata2(:,end-1:end));

figure(4);
lcpdata1=csvread([path 'lcp_data1.csv']);
hist(lcpdata1(:,end))
title('contact 1')

figure(5);
lcpdata2_trim=csvread([path 'lcp_data2_trim_.csv']);
subplot(2,1,1)
hist(lcpdata2_trim(:,end-1))
title('contact 1')
subplot(2,1,2)
hist(lcpdata2_trim(:,end))
title('contact 2')
% hist3(lcpdata2_trim(:,end-1:end));