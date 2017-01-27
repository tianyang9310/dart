clc,clear;
close all;

path='/Users/Yang/Material/Research/dart/build/data/'

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
lcpdata3=csvread([path 'lcp_data3.csv']);
subplot(2,2,1)
hist(lcpdata3(:,end-2))
title('contact 1')
subplot(2,2,2)
hist(lcpdata3(:,end-1))
title('contact 2')
subplot(2,2,3)
hist(lcpdata3(:,end))
title('contact 3')

figure(3);
lcpdata2=csvread([path 'lcp_data2.csv']);
subplot(2,1,1)
hist(lcpdata2(:,end-1))
title('contact 1')
subplot(2,1,2)
hist(lcpdata2(:,end))
title('contact 2')
hist3(lcpdata2(:,end-1:end));

figure(4);
lcpdata1=csvread([path 'lcp_data1.csv']);
hist(lcpdata1(:,end))
title('contact 1')
