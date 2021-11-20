load("training_data_rot.mat","act_pos","ee_pos","ee_rot");

act_pos = act_pos * 100;
ee_pos = ee_pos * 100;

addpath '/Users/avirudich/Dir/Robotics/ZoomLab-Projects/Delta/Prismatic Delta'
Prismatic_Delta_Params;

rigid_offset = norm(r.FK([0,0,0]) + .5);
z = (ee_pos(:,3)+rigid_offset) .* ones(size(ee_pos));
invalid = any(act_pos > z,2);

a_v = act_pos(~invalid,:);
z_v = z(~invalid,:);

[d_max,i] = max(max(a_v,[],2)-min(a_v,[],2));

