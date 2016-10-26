clc,clear;
close all;
figure(1);
lcpdata4=csvread('/tmp/lcp_data4.csv');
subplot(2,2,1)
hist(lcpdata4(:,91))
title('contact 1')
subplot(2,2,2)
hist(lcpdata4(:,92))
title('contact 2')
subplot(2,2,3)
hist(lcpdata4(:,93))
title('contact 3')
subplot(2,2,4)
hist(lcpdata4(:,94))
title('contact 4')

%% 
% clc,clear;
% close all;
figure(2);
lcpdata4=csvread('/tmp/M_lcp_data4.csv');
subplot(2,2,1)
hist(lcpdata4(:,91))
title('contact 1')
subplot(2,2,2)
hist(lcpdata4(:,92))
title('contact 2')
subplot(2,2,3)
hist(lcpdata4(:,93))
title('contact 3')
subplot(2,2,4)
hist(lcpdata4(:,94))
title('contact 4')