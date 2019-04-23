timestamps = {};
Quaternion = {};
ass_ground = load('ass_groundtruth_desk.mat');
load('xyz_hybrid.mat');
% abs_matrix = load('absMatrix_xyz.mat');
%abs_matrix = load('cvo_xyz_align_gt.mat')
fileID = fopen('Quat_xyz_hybrid.txt','w');
% fprintf(fileID,'%6s %12s\n','#timestamps','quat');
% desk
% temp_trans = [1.3112;0.8507;1.5186;1];
% xyz
temp_trans = [0; 0; 0; 1];
for i = 1:size(groundMatrix)
    num = num2str(i-1);
    name = 'num';
    s = strcat(name,num);
    timestamps{i} = ass_ground.fram_trans_dic.(s);
    % rotation matrix
%     Matrix_rot = abs_matrix.groundMatrix{i}(1:3,1:3);
    %% 
    Matrix_rot = groundMatrix{i}(1:3,1:3);
    % robot location
    trans = groundMatrix{i} * temp_trans;
    %%
    trans = trans(1:3);
    trans = trans';
    % quat
    quat_temp = abs(rotm2quat(Matrix_rot));
    quat = zeros(1,4);
    quat(1:3) = quat_temp(2:4);
    quat(3) = -quat(3);
    quat(4) = -quat_temp(1);
    Quaternion{i} = [trans,quat];
    
    fprintf(fileID,'%.4f', timestamps{i}); 
    fprintf(fileID,' %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n',Quaternion{i});
end

 fclose(fileID);
% save('transMatrix_xyz.mat','transformationMatrix')
% save('Quat_xyz.txt','timestamps','Quaternion')
