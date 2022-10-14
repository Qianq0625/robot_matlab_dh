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
robot.tool = transl(0.3,0,0); 

view(3)
robot.plot([0,0,0,0,0,0],'movie','Robot');

% trajectory
T=pos_static_trajectory';

% show coordinates in 3-D space 
plot3(T(:,1),T(:,2),T(:,3));
hold on

T4=transl(T);
% Inverse kinematics
q=robot.ikine(T4);
hold on
view(3)
% polt the trail of the end-effector
robot.plot(q,'trail', {'-r', 'LineWidth', 2},'movie','Task1.mp4' );
hold off


%workspace
    format short;
    degree=pi/180;  %degree
    radian=180/pi; 
    %Joint Angle limit
    q1_s=-180; q1_end=180;
    q2_s=-180;    q2_end=180;
    q3_s=-180;  q3_end=180;
    q4_s=-180; q4_end=180;
    q5_s=-180;  q5_end=180;
    q6_s=0;    q6_end=360;
    
    %number
    num=100000;
 
%% compute workspce
    %Set axis joint random distribution, axis 6 does not affect the working range, set to 0
    q1_rand = q1_s + rand(num,1)*(q1_end - q1_s);%rand generates a random num between 0 and 1
    q2_rand = q2_s + rand(num,1)*(q2_end - q2_s);
    q3_rand = q3_s + rand(num,1)*(q3_end - q3_s);
    q4_rand = q4_s + rand(num,1)*(q4_end - q4_s);
    q5_rand = q5_s + rand(num,1)*(q5_end - q5_s);
    q6_rand = rand(num,1)*0;
    q = [q1_rand q2_rand q3_rand q4_rand q5_rand q6_rand]*degree;
    
    % Forward kinematics calculation workspace
    tic;
    T_cell = cell(num,1);
    for i=1:1:num
        T_cell{i}=robot.fkine(q(i,:));
    end
    disp(['Work time：',num2str(toc)]);

    %plot workspce
    t1=clock;
    figure('name','Robot Workspce')
    hold on
     figure_x=zeros(num,1);
     figure_y=zeros(num,1);
     figure_z=zeros(num,1);
     for cout=1:1:num
         figure_x(cout,1)=T_cell{cout,1}.t(1);
         figure_y(cout,1)=T_cell{cout,1}.t(2);
         figure_z(cout,1)=T_cell{cout,1}.t(3);
     end
     plot3(figure_x,figure_y,figure_z,'r.','MarkerSize',3);
     hold off
     disp(['Workspce time：',num2str(etime(clock,t1))]);  
     
     %X Y Z
     Point_range=[min(figure_x) max(figure_x) min(figure_y) max(figure_y) min(figure_z) max(figure_z)];
