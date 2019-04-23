d = 791;

dd = 231;
% read groundtruth
GT_id = fopen('ass_groundtruth_xyz.txt','r'); % xyz
% GT_id = fopen('ass_groundtruth_desk.txt','r'); % desk
% GT_id = fopen('s_dvo_assG_RGB_xyz','r'); % sDvo xyz
% GT_id = fopen('s_dvo_assG_RGB_desk.txt','r'); % sDvo desk
formatSpec = '%f';
sizeA = [8 d];
GT_pose = fscanf(GT_id,formatSpec,sizeA);
GT_pose =GT_pose';
fclose(GT_id);
% read est
Est_id_cvo = fopen('Direct_Quat_xyz_cvo.txt','r'); % xyz
formatSpec = '%f';
Est_pose_cvo = fscanf(Est_id_cvo,formatSpec,sizeA);
Est_pose_cvo = Est_pose_cvo';
fclose(Est_id_cvo);

Est_id_dvo = fopen('Direct_Quat_xyz_dvo.txt','r'); % xyz
Est_pose_dvo = fscanf(Est_id_dvo,formatSpec,sizeA);
Est_pose_dvo = Est_pose_dvo';
fclose(Est_id_dvo);

Est_id_color = fopen('Direct_Quat_xyz_color.txt','r'); % xyz
Est_pose_color = fscanf(Est_id_color,formatSpec,sizeA);
Est_pose_color = Est_pose_color';
fclose(Est_id_color);

Est_id_hybrid = fopen('Direct_Quat_xyz_hybrid.txt','r'); % xyz
Est_pose_hybrid = fscanf(Est_id_hybrid,formatSpec,sizeA);
Est_pose_hybrid = Est_pose_hybrid';
fclose(Est_id_hybrid);

GT_sDvo = fopen('s_dv0_assG_RGB_xyz.txt','r'); % xyz
sizeB = [9 dd];
GT_pose_sDvo = fscanf(GT_sDvo,formatSpec,sizeB);
GT_pose_sDvo =GT_pose_sDvo';
fclose(GT_sDvo);
Est_id_sDvo = fopen('Direct_Quat_xyz_sDvo.txt','r'); % xyz
sizeC = [8 dd];
Est_pose_sDvo = fscanf(Est_id_sDvo,formatSpec,sizeC);
Est_pose_sDvo = Est_pose_sDvo';
fclose(Est_id_sDvo);


ATE_cvo = [];
ATE_dvo = [];
ATE_color = [];
ATE_hybrid = [];
ATE_sDvo = [];

GT = zeros(3,d);
Est_cvo = ones(3,d);
Est_dvo = zeros(3,d);
Est_color = zeros(3,d);
Est_hybrid = zeros(3,d);
Est_sDvo = zeros(3,d);

for i = 1:d
    GT_temp = GT_pose(i,2:4);
    GT(:,i) = GT_temp';
    Est_temp_cvo = Est_pose_cvo(i,2:4);
    Est_cvo(:,i) = Est_temp_cvo';
    ATE_cvo(i) = (GT_temp(1)-Est_temp_cvo(1))^2+(GT_temp(2)-Est_temp_cvo(2))^2+ (GT_temp(3)-Est_temp_cvo(3))^2;
    ATE_cvo(i) = ATE_cvo(i)^0.5;
    
    Est_temp_dvo = Est_pose_dvo(i,2:4);
    Est_dvo(:,i) = Est_temp_dvo';
    ATE_dvo(i) = (GT_temp(1)-Est_temp_dvo(1))^2+(GT_temp(2)-Est_temp_dvo(2))^2+ (GT_temp(3)-Est_temp_dvo(3))^2;
    ATE_dvo(i) = ATE_dvo(i)^0.5;
    
    Est_temp_color = Est_pose_color(i,2:4);
    Est_color(:,i) = Est_temp_color';
    ATE_color(i) = (GT_temp(1)-Est_temp_color(1))^2+(GT_temp(2)-Est_temp_color(2))^2+ (GT_temp(3)-Est_temp_color(3))^2;
    ATE_color(i) = ATE_color(i)^0.5;
    
    Est_temp_hybrid = Est_pose_hybrid(i,2:4);
    Est_hybrid(:,i) = Est_temp_hybrid';
    ATE_hybrid(i) = (GT_temp(1)-Est_temp_hybrid(1))^2+(GT_temp(2)-Est_temp_hybrid(2))^2+ (GT_temp(3)-Est_temp_hybrid(3))^2;
    ATE_hybrid(i) = ATE_hybrid(i)^0.5;
end
for i = 1:dd
    GT_temp_sDvo = GT_pose_sDvo(i,2:4);
    Est_temp_sDvo = Est_pose_sDvo(i,2:4);
    Est_sDvo(:,i) = Est_temp_sDvo';
    ATE_sDvo(i) = (GT_temp_sDvo(1)-Est_temp_sDvo(1))^2+(GT_temp_sDvo(2)-Est_temp_sDvo(2))^2+ (GT_temp_sDvo(3)-Est_temp_sDvo(3))^2;
    ATE_sDvo(i) = ATE_sDvo(i)^0.5;
end

% plot3(GT(1,:),GT(2,:),GT(3,:),'b')
% hold on
% plot3(Est(1,:),Est(2,:),Est(3,:),'m')
% legend({'y = groundtruth','y = est. trajectory'},'Location','southwest')
% hold off
% plot(GT(1,:),GT(2,:),'b')
% hold on
% plot(Est(1,:),Est(2,:),'m')
% legend({'y = groundtruth','y = est. trajectory'},'Location','southwest')
% hold off

figure(1)
hold on
plot(sort(ATE_cvo), [1:d]/(d),'color','#D95319','MarkerSize',10)
plot(sort(ATE_dvo), [1:d]/(d),'color','r','MarkerSize',10)
plot(sort(ATE_sDvo), [1:dd]/(dd),'color','#7E2F8E','MarkerSize',10)
plot(sort(ATE_color), [1:d]/(d),'color','#77AC30','MarkerSize',10)
plot(sort(ATE_hybrid), [1:d]/(d),'color','#0072BD','MarkerSize',10)
hold off
title('fr1/xyz')
xlabel('position error') 
ylabel('fraction of data') 
legend({'cvo','dvo','sDvo','open3d color','open3d hybrid'},'Location','southeast')

