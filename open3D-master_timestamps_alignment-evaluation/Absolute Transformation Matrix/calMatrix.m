load('fram_trans_xyz_hybrid.mat')
transformationMatrix = {};
groundMatrix = {};
Matrix = {};
% xyz
Matrix_temp = [0.0754, 0.6139,  -0.7857, 1.3405;
                0.9971, -0.0384, 0.0657,  0.6266;
                0.0102, -0.7884, -0.6150, 1.6575;
                0,      0,       0,       1];
% desk
% Matrix_temp = [0.8723    0.3479   -0.3436  1.3112;
%                0.4883   -0.5828    0.6495  0.8507;
%                0.0257   -0.7343   -0.6783  1.5186;
%                0         0         0       1];
% desk sDvo
% Matrix_temp = [0.9119    0.2681   -0.3108   1.3480
%                0.4104    -0.5996   0.6870   0.8158
%                -0.0022   -0.7540   -0.6568  1.5311
%                0         0         0        1];
% xyz sDvo
% Matrix_temp = [ 0.0080    0.7018   -0.7123   1.1372
%                 0.9967   -0.0634   -0.0513   0.6225
%                 -0.0812   -0.7095   -0.7000  1.4315
%                 0         0         0        1];
for i = 1:791
% desk
% for i = 1:572
    num = num2str(i-1);
    name = 'num';
    s = strcat(name,num);
    transformationMatrix{i} = fram_trans_dic.(s);
    Matrix_temp = Matrix_temp * fram_trans_dic.(s);
    groundMatrix{i} = Matrix_temp;
%     transformationMatrix{i} = frame_transform{i,2};
%     Matrix_temp = Matrix_temp * frame_transform{i,2};
%     groundMatrix{i} = Matrix_temp;
end

save('xyz_sDvo.mat','groundMatrix')
    
