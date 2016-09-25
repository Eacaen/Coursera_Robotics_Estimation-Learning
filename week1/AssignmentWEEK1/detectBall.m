% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [S, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 
 
% load('mu.mat');
% load('sig.mat');
 thre = 0;

 mu =  [ 150.7131  145.4511   59.8629];
%sig = diag([180.8987  128.4632  339.5755]);
sig =  [  1.6176    0   0;
              0   5.2613    0;
             0    0    2.5000];

[n,m,s] = size(I);
bw = zeros(n,m);
for i=1:n
    for j =1:m
       % pro = mvnpdf(double([I(i,j,1) I(i,j,2) I(i,j,3)]),mu,sig);
       data = double( [I(i,j,1) I(i,j,2) I(i,j,3)] ) - mu;
       A = data * inv(sig) ;
       A = A * data';
       pro = 1/( (2*pi)^1.5 *  (det(sig) )^0.5) * exp( -0.5 * A);
        if pro > thre
            bw(i,j) = 1;
        end
    end
end
%figure, imshow(bw); 

bw_biggest = false(size(bw));

CC = bwconncomp(bw);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true;  
%figure,imshow(bw_biggest); hold on;

% show the centroid
% http://www.mathworks.com/help/images/ref/regionprops.html
S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
S = bw_biggest;
%figure, imshow(S);hold on;
%plot(loc(1), loc(2),'r+'); %此处plot是在上一个figure上面作图

end