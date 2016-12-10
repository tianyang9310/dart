image=csvread('image.txt');
truth=csvread('truth.txt');
image = image';
% plot(image)
% plot(truth)
truth = truth';
theta = (image * image')\(image * truth')
(theta' * image);