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

% Initialise variables
count = 1;
T3 = transl(pos_beat_trajectory');

for t=1:length(T3) 
    % Compute joint positions and velocities
    if (t==1)
        % Initial joint configuration
        qjcb(:,count)=robot.ikine(T3(:,:,count));
    else
        %V=Jacob*W(Articular angular velocity)
        %W=V*(inverse_J)
        %(New Joint angle)w=(Previous joint angle)w0+W*t=w0+V*(inverse_J)*t         
        syms Vx Vy t
        V=0.1;       
        x=T3(1,4,count)-T3(1,4,count-1);
        y=T3(2,4,count)-T3(2,4,count-1);
        e1 = Vx-x/t;
        e2 = Vy-y/t;      
        e3=V-sqrt(Vx^2+Vy^2);
        [Vx,Vy,t]=solve(e1,e2,e3,Vx,Vy,t);%solve    
      
        if x > 0
            vx = abs(double(Vx(1,1)));
        else
            vx = -abs(double(Vx(1,1)));
        end
 
        if y > 0
            vy = abs(double(Vy(1,1)));
        else
            vy = -abs(double(Vy(1,1)));
        end
        
        t1 = abs(double(t(1,1)));
        
        %w=w0+W*t=w0+V*(inverse_J)*t
        J0=robot.jacob0(qjcb(:,count-1));
        v=[vx;vy;0;0;0;0];       
        qjcb(:,count)=qjcb(:,count-1)+(pinv(J0(:,:)))*v*t1;
        
    end
    count=count+1;
end

plot3(squeeze(T3(1,4,:)),squeeze(T3(2,4,:)),squeeze(T3(3,4,:)),'b-');
hold on;
view(3);
robot.plot(qjcb','trail', {'-r', 'LineWidth', 2},'movie','T3a.mp4');
hold off


%% T3B
R4 = zeros(3,3,length(pos_beat_trajectory));
T4=zeros(4,4,length(pos_beat_trajectory));
co=zeros(29,1);
for i=1:length(T4)
    R4(:,:,i) = angvec2r(alpha(1,i),vector);
    T4(:,:,i) = rt2tr(R4(:,:,i), pos_beat_trajectory(:,i));
    co(i)=acos(R4(1,1,i));
end


count = 1;

T3 = transl(pos_beat_trajectory');


for t=1:length(T3) 
    % Compute joint positions and velocities
    if (t==1)
        % Use inverse kinematics to determine initial configuration
        % Initial joint configuration
        qjcb(:,count)=robot.ikine(T3(:,:,count));
    else       
        syms Vx Vy t
        V=0.1;       
        x=T3(1,4,count)-T3(1,4,count-1);
        y=T3(2,4,count)-T3(2,4,count-1);
        e1 = Vx-x/t;
        e2 = Vy-y/t;      
        e3=V-sqrt(Vx^2+Vy^2);
        [Vx,Vy,t]=solve(e1,e2,e3,Vx,Vy,t);
      
        if x > 0
            vx = abs(double(Vx(1,1)));
        else
            vx = -abs(double(Vx(1,1)));
        end
        
 
        if y > 0
            vy = abs(double(Vy(1,1)));
        else
            vy = -abs(double(Vy(1,1)));
        end
        
        t1 = abs(double(t(1,1)));
        wz=(co(count)-co(count-1))/t1;
        
        %w0+W*t=w0+V*(inverse_J)*t
        J0=robot.jacob0(qjcb(:,count-1));
        v=[vx;vy;0;0;0;wz];       
        qjcb(:,count)=qjcb(:,count-1)+(pinv(J0(:,:)))*v*t1;
        
    end
    count=count+1;
end

for i=1:size(qjcb,1)
    for j=1:size(qjcb,2)
        if(abs(qjcb(i,j)) <= 0.001)
            qjcb(i,j) = 0;
        end
    end
end

plot3(squeeze(T3(1,4,:)),squeeze(T3(2,4,:)),squeeze(T3(3,4,:)),'b-');
hold on;
view(3);
robot.plot(qjcb','trail', {'-r', 'LineWidth', 2},'movie','T3b.mp4');
hold off


