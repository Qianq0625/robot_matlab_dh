clc;clear;
load('trajectory_data.mat');

%DH
L1=Link('d',0.1,'a',0,'alpha',0,'modified','qlim',[-pi*2,pi*2]);
L2=Link('d',0,'a',0.4,'alpha',pi/2,'offset',pi/2,'modified','qlim',[-pi*2,pi*2]);
L3=Link('d',0,'a',0.8,'alpha',0,'offset',0,'modified','qlim',[-pi*2,pi*2]);
L4=Link('d',0.8,'a',0,'alpha',pi/2,'offset',pi,'modified','qlim',[-8*pi/9,8*pi/9]);
L5=Link('d',0,'a',0,'alpha',pi/2,'offset',0,'modified','qlim',[-pi*2,pi*2]);
L6=Link('d',0,'a',0.4,'alpha',-pi/2,'modified','qlim',[-pi*2,pi*2]);
%L7=Link('d',0,'a',0.3,'alpha',0,'modified','qlim',[0,0]);
robot=SerialLink([L1,L2,L3,L4,L5,L6],'name','TaskRobot');
robot.tool = transl(0.3,0,0); %
view(3)
%robot.teach
%robot.plot([0.0.0.0.0.0.0]);
robot.plot([0,0,0,0,0,0],'movie','Robot');

% trajectory
T=pos_static_trajectory';

% show coordinates in 3-D space 
plot3(T(:,1),T(:,2),T(:,3));
hold on

T1=transl(T);
% Inverse kinematics
q=robot.ikine(T1);
hold on
view(3)
% polt the trail of the end-effector
robot.plot(q,'trail', {'-r', 'LineWidth', 2},'movie','Task1.mp4' );
hold off

rotation = zeros(3,3,length(pos_static_trajectory));
T2=zeros(4,4,length(pos_static_trajectory)); 
for i=1:length(T2)
    rotation(:,:,i) = angvec2r(alpha(1,i),vector);   % Convert angle and vector orientation to a rotation matrix
    T2(:,:,i) = rt2tr(rotation(:,:,i), pos_static_trajectory(:,i));  % Convert rotation and translation to homogeneous transform
end
plot3(squeeze(T2(1,4,:)),squeeze(T2(2,4,:)),squeeze(T2(3,4,:)));

hold on
qp=robot.ikine(T2);  % Inverse kinematic
view(3)
robot.plot(qp,'trail', {'-r', 'LineWidth', 2},'movie','Task2.mp4');
hold off
