


%%%%%%%%%%%%%%%%%@copy by sobhan siamak %%%%%%%%%%%%%
clc;
clear;
close all;

img=imread('img.jpg');
figure,
imshow(img);
title('Original Image');
img2=rgb2gray(img);
img2=im2double(img2);


%% Canny Edge Detection
figure,
imshow(img2,[]);
E=edge(img2,'canny',0.1);
figure,
imshow(E);

%% Hough Transform

% HL=[-1 -1 -1
%     2 2 2 
%     -1 -1 -1];
% 
% horizon=imfilter(img,HL);
% figure,
% imshow(horizon);
% 
% vertical=imfilter(img,HL');
% figure,
% imshow(vertical);
% 


% [H, theta, rho] = hough(E);
% peaks = houghpeaks(H, 10);
% 
% for i=1:size(peaks,1)
%        % Extract rho, theta for this line
%         r = rho(peaks(i,1));
%         t = theta(peaks(i,2));
%         % Equation of the line is r = x cos(t) + y sin(t), or
%         % y = (r - x*cos(t))/sin(t)
%         x0 = 1;
%         y0 = (r - x0*cosd(t))/sind(t);
%         x1 = size(img,2);
%         y1 = (r - x1*cosd(t))/sind(t);
%         line([x0 x1], [y0 y1], 'Color', 'r');
% end


[H, theta, rho]= HoughLine(E);%%%%%My Function
[H1,theta1,rho1] = hough(E);%%%%%Matlab Function

g= uint8(H);
h= uint8(H1);

figure,
subplot(1,2,1), imshow(g),title('My H image');
subplot(1,2,2), imshow(h),title('Using inbuit function');




Threshold=0.5 * max(H(:));
NHoodSize= (floor(size(H) / 100.0) * 2 + 1);
peaks = HPeaks(H,10);
peaks1 = houghpeaks(H,10);

figure,
imshow(H,[],'XData',theta,'YData',rho,'InitialMagnification','fit');
xlabel('\theta'), ylabel('\rho'), title('Hough Transform Peaks');
axis on, axis normal, hold on;
plot(theta(peaks(:,2)),rho(peaks(:,1)),'s','color','white');



 HLines(img, peaks, rho, theta);
 
 
 %% Hough Transform with Noise

imagenoise=imnoise(img,'gaussian',0,0.07);%%%%%mean=0 and var=0.2
figure,
imshow(imagenoise);
img3=rgb2gray(imagenoise);
img3=im2double(img3);



E2=edge(img3,'canny',0.3);
figure,
imshow(E2);


[Hn, thetan, rhon]= HoughLine(E2);%%%%%My Function
[Hn1,thetan1,rhon1] = hough(E2);%%%%%Matlab Function

g2= uint8(Hn);
h2= uint8(Hn1);



figure,
subplot(1,2,1), imshow(g2),title('My H image');
subplot(1,2,2), imshow(h2),title('Using inbuit function');



Threshold=0.5 * max(Hn(:));
NHoodSize= (floor(size(Hn) / 100.0) * 2 + 1);
peaksn = HPeaks(Hn,10);
peaksn1 = houghpeaks(Hn,10);

figure,
imshow(Hn,[],'XData',thetan,'YData',rhon,'InitialMagnification','fit');
xlabel('\theta'), ylabel('\rho'), title('Hough Transform Peaks');
axis on, axis normal, hold on;
plot(thetan(peaks(:,2)),rhon(peaks(:,1)),'s','color','white');



 HLines(imagenoise, peaksn, rhon, thetan);









