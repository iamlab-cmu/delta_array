load("Delta_1_Kinematic_Test_Flat.mat","ee_pos_1","ee_pos_2","ee_pos_3","ee_pos_aug","ee_pos_base","ee_pos_rigid","traj");

var = ee_pos_aug;

error = vecnorm(var'-traj');
m1 = mean(error)
s1 = std(error)

subplot(2,3,1);
easy_3d_stem(traj,get_color(1));
title("Delta 1");
hold on
easy_3d_stem(var,get_color(7));
axis([-4.5,4.5,-4.5,4.5,1,3]);
hold off
view([0,0]);

subplot(2,3,4);
easy_3d_stem(traj,get_color(1));
hold on
easy_3d_stem(var,get_color(7));
axis([-4.5,4.5,-4.5,4.5,1,3]);
hold off
view(2);

load("Delta_2_Kinematic_Test_Flat.mat","ee_pos_1","ee_pos_2","ee_pos_3","ee_pos_aug","ee_pos_base","ee_pos_rigid","traj");

var = ee_pos_aug;

error = vecnorm(var'-traj');
m2 = mean(error)
s2 = std(error)

subplot(2,3,2);
easy_3d_stem(traj,get_color(1));
title("Delta 2");
hold on
easy_3d_stem(var,get_color(7));
axis([-4.5,4.5,-4.5,4.5,1,3]);
hold off
view([0,0]);

subplot(2,3,5);
easy_3d_stem(traj,get_color(1));
hold on
easy_3d_stem(var,get_color(7));
axis([-4.5,4.5,-4.5,4.5,1,3]);
hold off
view(2);

load("Delta_3_Kinematic_Test_Flat.mat","ee_pos_1","ee_pos_2","ee_pos_3","ee_pos_aug","ee_pos_base","ee_pos_rigid","traj");

var = ee_pos_aug;
error = vecnorm(var'-traj');
m3 = mean(error)
s3 = std(error)

subplot(2,3,3);
easy_3d_stem(traj,get_color(1));
title("Delta 3");
hold on
easy_3d_stem(var,get_color(7));
axis([-4.5,4.5,-4.5,4.5,1,3]);
hold off
view([0,0]);

subplot(2,3,6);
easy_3d_stem(traj,get_color(1));
hold on
easy_3d_stem(var,get_color(7));
axis([-4.5,4.5,-4.5,4.5,1,3]);
hold off
view(2);

f=gcf;
f.Position = [100 100 1040 800];
