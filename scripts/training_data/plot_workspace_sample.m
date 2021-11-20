load("training_data_rot.mat","act_pos","ee_pos","ee_rot");

act_pos = act_pos * 100;
ee_pos = ee_pos * 100;

addpath '/Users/avirudich/Dir/Robotics/ZoomLab-Projects/Delta/Prismatic Delta'
Prismatic_Delta_Params;

rigid_sample = r.FK_Traj(act_pos);
rigid_offset = norm(r.FK([0,0,0]) + [0,0,.5]);
ee_pos(:,3) = ee_pos(:,3)+rigid_offset;

[m1,m2,m3,A_ideal,E_ideal,valid_mask,E_sim] = Delta_With_Offsets_Metrics_Analytical(1,act_pos);

m_off = mean(vecnorm(ee_pos(valid_mask~=0,:)'-E_sim(valid_mask~=0,:)'))
s_off = std(vecnorm(ee_pos(valid_mask~=0,:)'-E_sim(valid_mask~=0,:)'))

return

m_ideal = mean(vecnorm(ee_pos'-rigid_sample'));
s_ideal = std(vecnorm(ee_pos'-rigid_sample'));



subplot(1,2,1);
easy_3d_stem(rigid_sample,get_color(1));
hold on
easy_3d_stem(ee_pos,get_color(7));
view([0,0]);
axis square
subplot(1,2,2);
easy_3d_stem(rigid_sample,get_color(1));
hold on
easy_3d_stem(ee_pos,get_color(7));
view(2);
axis square
hold off
sgtitle("Rigid Delta with Offsets Workspace vs. Sampled Flexible Delta Workspace");
