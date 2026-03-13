% 采用均值滤波、高斯滤波、中值滤波分别处理企鹅图片，通过模板大小、高斯核等参数设置，对比三种滤波器的效果。
% 读取图片
clc; clear; close all;
%% 读取图像
I = imread('图片1.png');    
I0 = rgb2gray(I);         
In = im2double(I0);
%% 参数设置（可以多改几组做对比）
meanK = 5;                   % 均值滤波模板大小：3/5/7...
gaussK = 5;  sigma = 0.8;    % 高斯核大小 & sigma
medianK = 5;                 % 中值滤波窗口大小：3/5/7...

%% 1) 均值滤波
h_mean = fspecial('average', [meanK meanK]);
Out_mean = imfilter(In, h_mean, 'replicate');

%% 2) 高斯滤波
Out_gauss = imgaussfilt(In, sigma, 'FilterSize', gaussK);

%% 3) 中值滤波
Out_median = medfilt2(In, [medianK medianK], 'symmetric');

%% 可视化对比
figure('Color','w','Name','滤波效果对比');
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

nexttile; imshow(In, []); title('输入(含噪)');
nexttile; imshow(Out_mean, []);   title(sprintf('均值滤波 %dx%d', meanK, meanK));
nexttile; imshow(Out_gauss, []);  title(sprintf('高斯滤波 %dx%d, \\sigma=%.2f', gaussK, gaussK, sigma));
nexttile; imshow(Out_median, []); title(sprintf('中值滤波 %dx%d', medianK, medianK));

% 采用混合增强法，选取学过的图像处理方式，对医学影像图片进行处理，解释并说明所选方法与顺序的原因，如有局限也可以阐述。

%%
% 读取图像
I = imread('图片2.png');                 
I = rgb2gray(I);
I = im2double(I);
%% 步骤1：空间滤波（中值滤波）  
k = 3;                                  
I1 = medfilt2(I, [k k], 'symmetric');

%% 步骤2：灰度变换（伽马变换）调整对比度
gamma = 1.2;                            
I2 = I1 .^ gamma;

%% 步骤3：空间滤波（拉普拉斯锐化）增强边缘 
alpha = 0.2;                           
h = fspecial('laplacian', alpha);
L = imfilter(I2, h, 'replicate');

% 拉普拉斯锐化
I3 = I2 - L;

I3 = imadjust(mat2gray(I3));

%% 显示对比
figure('Color','w','Name','第2题：中值去噪 + 伽马变换 + 拉普拉斯锐化');
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

nexttile; imshow(I,  []); title('原始X光图像');
nexttile; imshow(I1, []); title(sprintf('步骤1：中值滤波 %dx%d', k, k));
nexttile; imshow(I2, []); title(sprintf('步骤2：伽马变换 \\gamma=%.2f', gamma));
nexttile; imshow(I3, []); title(sprintf('步骤3：拉普拉斯锐化 \\alpha=%.2f', alpha));