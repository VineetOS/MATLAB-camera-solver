% Real Time Autonomous Systems Programming Assignment 1
% Oswal Vinit Jitendra (B18ME040)
% Tarun Meena (B18ME056)


% Given data
% Intrinsic Matrix
load Intrinsic_Matrix_K.txt
K = Intrinsic_Matrix_K;


% Q1. Ground truth corresponences calculations
I1 = rgb2gray(imread('im1.jpg'));
I2 = rgb2gray(imread('im2.jpg'));

points1 = detectHarrisFeatures(I1);
points2 = detectHarrisFeatures(I2);

[features1,valid_points1] = extractFeatures(I1,points1);
[features2,valid_points2] = extractFeatures(I2,points2);

indexPairs = matchFeatures(features1,features2);

matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);

m1 = matchedPoints1.Location;
m2 = matchedPoints2.Location;

% Q2. Essential matrix "E" calculations

% m1 and m2 are ground truth correspondence matrics 
s = length(m1);
m1=[m1(:,1) m1(:,2) ones(s,1)];
m2=[m2(:,1) m2(:,2) ones(s,1)];

x1=m1'; x2=m2';
x1=[x1(1,:)' x1(2,:)'];  
x2=[x2(1,:)' x2(2,:)'];

A=[x1(:,1).*x2(:,1) x1(:,2).*x2(:,1) x2(:,1) x1(:,1).*x2(:,2) x1(:,2).*x2(:,2) x2(:,2) x1(:,1) x1(:,2), ones(s,1)];

[U D V] = svd(A);
E=reshape(V(:,9), 3, 3)';

[U D V] = svd(E);
d = (D(1,1) + D(2,2))/2;
E=U*diag([d d 0])*V';
disp(E)

% Q3. "R" and "t" vector calculations

[U D V] = svd(E); 

theta1 = pi/2;
theta2 = -pi/2;

% rotation matrix about Z-axis

Rz1 = [ cos(theta1) sin(theta1) 0;
        sin(theta1) cos(theta1) 0;
        0 0 1                    ];

Rz2 = [ cos(theta2) sin(theta2) 0;
        sin(theta2) cos(theta2) 0;
        0 0 1                    ];
    
    
R1 = U*transpose(Rz1)*transpose(V);
R2 = U*transpose(Rz2)*transpose(V);

% tx1 = U*Rz1*D*transpose(U);
% tx2 = U*Rz2*D*transpose(U);

% t1 = tx1(:,3);
% t2 = tx2(:,3);
t1 = U(:,3);
t2 = -U(:,3);

% Calculate the corresponding camera matrices for camera 1 and camera 2

% The world co-ordinate system is assumed to be at first camera center
P1 = [eye(3) zeros(3,1)];

% Calculating for rotation and translation w.r.t. camera 1
% Choosing the correct matrix by hit and trial of the 4 combinations

% P2 = [R1 t1];
% P2 = [R1 t2];
P2 = [R2 t2];
% P2 = [R2 t1];

% calculating camera matrics
M = K*P1 ; 
N = K*P2 ;

% Q4. 3D points calculations by triangulation approach
Xw = triangulation(m1',K*P1, m2',K*P2);

% Q5. "Pi" plot w.r.t. camera center
xx=Xw(1,:);
yy=Xw(2,:);
zz=Xw(3,:);

figure(1);
plot3(xx, yy, zz, 'bo');
title('3D points of the image w.r.t. camera center')
xlabel('X-axis')
ylabel('Y-axis')
zlabel('Z-axis')
grid on




function Xw = triangulation(x1,P1,x2,P2)

    x11 = x1; 
    x22 = x2; 


    for i=1:size(x11,2)
        sx1 = x11(:,i);
        sx2 = x22(:,i);
    
    
        A1 = sx1(1,1).*P1(3,:) - P1(1,:);
        A2 = sx1(2,1).*P1(3,:) - P1(2,:);
        A3 = sx2(1,1).*P2(3,:) - P2(1,:);
        A4 = sx2(2,1).*P2(3,:) - P2(2,:);
    
        A = [A1;A2;A3;A4];
        [U,D,V] = svd(A);
    
        X_temp = V(:,4);
        X_temp = X_temp ./ repmat(X_temp(4,1),4,1);
    
        Xw(:,i) = X_temp;
        
    end
end
