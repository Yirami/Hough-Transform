clear;
clc;
%%
image_edge = imread("../media/debug_edge.png");
[H,theta,rho] = hough(image_edge);
peaks = houghpeaks(H,10);
lines = houghlines(image_edge,theta,rho,peaks);
hmat = csvread("../media/hmat.csv");
hmat=hmat(:,1:180);
delta = H-hmat;
subplot(1,3,1);
mesh(H);
title('H');
subplot(1,3,2);
mesh(hmat);
title('hmat');
subplot(1,3,3);
mesh(delta);
title('delta')
%%