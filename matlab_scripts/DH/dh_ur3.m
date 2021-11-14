close all
clc

name = 'UR3';
nj = 6;

%https://www.universal-robots.com/how-tos-and-faqs/faq/ur-faq/parameters-for-calculations-of-kinematics-and-dynamics-45257/
% Different because we changed the orientation of the base X axis
% Nominal Denavit-Hartenberg parameters and link distances
lf = [0.1519 0.24365 0.21325 0.11235 0.08535 0.0819];
lexp = [0 0 0 0 0 0];
lf = lf*1000;
% DH: [a, alpha,    d, theta] 
dh_f = [ 0    pi/2	 lf(1)    0;
       -lf(2)   0      0      0;
       -lf(3)   0      0      0;
        0    pi/2    lf(4)    0;
        0    -pi/2   lf(5)    0;
        0     0      lf(6)    0]

plot = true;
  


[lh, c, r] = main(name, nj,plot);

lexp(1) = abs(c(3,2));
lexp(2) = abs(c(1,2) - c(1,3));
lexp(3) = abs(c(3,3) - c(3,4));
lexp(4) = abs(c(2,4));
lexp(5) = abs(c(1,4) - c(1,5));
lexp(6) = abs(c(3,5) - c(3,6));
lexp = lexp*1000;
dh_exp = [   0       pi/2	 lexp(1)    0;
       -lexp(2)    0       0        0;
       -lexp(3)    0       0        0;
          0       pi/2   lexp(4)    0;
          0      -pi/2   lexp(5)    0;
          0        0     lexp(6)    0]

dh_erro = dh_exp-dh_f
centers = c
centers(:,1) = [0;c(2,2);0];
centers1 = [centers(1,5) centers(1,5); centers(2,5) 0 ;centers(3,5) centers(3,5)];
hold on;
plot3(centers(1,:), centers(2,:), centers(3,:),'color',[0.9100    0.4100    0.1700],'LineWidth',2);
plot3(centers1(1,:), centers1(2,:), centers1(3,:),'color',[0.9100    0.4100    0.1700],'LineWidth',2);
[x y] = meshgrid(-10:1:10);
z = 0.*x + 0.*y + 0;
surf(x,y,z);

lgd = legend(lh, 'J1', 'J2', 'J3', 'J4', 'J5', 'J6');
lgd.FontSize = 20;
