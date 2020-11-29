%% Problem 3 (Part 2b)
clc;close all; clear all;
% Constants
p = -5:0.01:5;
c = -5:0.01:5;
[X,Y] = meshgrid(p,c);
l=0.07; 
dt=0.001; % step size or sampling frequency
tf=7; % Final time
iterations=length(0:dt:tf); %Number of iterations
R = [0, -1; 1, 0];
k1 = 3;
k2 = 1;
zd = 2;

% Initialization
r1_des = zeros(2,iterations);
r1_des(1,1) = 3; %Initial Position
r1_des(2,1) = 1; %Initial Position
r2_des = zeros(2,iterations);
r2_des(1,1) = 4; %Initial Position
r2_des(2,1) = 1; %Initial Position

x1_0 = r1_des(1,1);
y1_0 = r1_des(2,1); %-0.1; ??
theta1_0 = pi/2;
x1 = zeros(1,iterations);
y1 = zeros(1,iterations);
theta1 = zeros(1,iterations);
x1(1,1) = x1_0;
y1(1,1) = y1_0;
r0 = [-2,0]';
theta1(1,1)=theta1_0;

x2_0 = r2_des(1,1);
y2_0 = r2_des(2,1); %-0.1; ??
theta2_0 = pi/2;
x2 = zeros(1,iterations);
y2 = zeros(1,iterations);
theta2 = zeros(1,iterations);
x2(1,1) = x2_0;
y2(1,1) = y2_0;
r0 = [-2,0]';
r1 = [3 1];
r2 = [4 1];
theta2(1,1)=theta2_0;





Z = zeros(length(p),length(c));
for k = 1:length(p)
    for j = 1:length(c)
        Z(k,j) = (p(k)-r0(1))^2 + (c(j)-r0(2))^2; %Scalar Field
    end
end
contour(X,Y,Z'); hold on
plot(r0(1),r0(2),'ro'); %Plot source in red

% prepare video
video_flag = 1;
if video_flag
    fig = figure(1);
    %pause;
    myVideo = VideoWriter('animation3_2b.mp4', 'MPEG-4');
    myVideo.FrameRate = 100;
    myVideo.Quality = 75;
    open(myVideo);
end

% prep plot
xMIN = -5;
xMAX = 5;
yMIN = -5;
yMAX = 5;

xlim([xMIN xMAX])
ylim([yMIN yMAX])
contour(X,Y,Z'); hold on
plot(r0(1),r0(2),'ro'); %Plot source in red

for i=2:iterations

    % Plotting
    if(i<30 || (i<45 && mod(i,2)==0) || mod(i,30)==0)
        fig=figure(1); clf;
        pose=[x1(1,i-1);y1(1,i-1);theta1(1,i-1)];
        pose2 = [x2(1,i-1);y2(1,i-1);theta2(1,i-1)];
        contour(X,Y,Z');
        plot_robot(pose,fig) %Plot the robot location
        plot_robot(pose2,fig) %Plot the robot location
        plot(r1(1),r1(2),'go')
        plot(r2(1),r2(2),'go')
        plot(r0(1),r0(2),'bo')
        plot(x1(1,1:i-1),y1(1,1:i-1))
        plot(x2(1,1:i-1),y2(1,1:i-1))
        if video_flag
            frame = getframe(gcf);
            writeVideo(myVideo, frame);
        end
    end
    %plot(x(1,i-1),y1(1,i-1),'bo','MarkerFaceColor','r');
    %plot(r1_des(1,i-1), r1_des(2,i-1), 'ro', 'MarkerFaceColor','r'); hold on;%Plot the goal
%     Z = zeros(length(p),length(c));
%     for k = 1:length(p)
%         for j = 1:length(c)
%             Z(k,j) = (p(k)-r0(1))^2 + (c(j)-r0(2))^2; %Scalar Field
%         end
%     end
%     contour(X,Y,Z'); hold off
    
    % Desired Trajectory
    w = (r1_des(:,i-1) - r2_des(:,i-1)) / norm(r1_des(:,i-1) - r2_des(:,i-1));
    v = R*w;
    r1dot_des = k1*((norm(r1_des(:,i-1)-r0))^2-zd)*v + k2*w;
    r2dot_des = k1*((norm(r2_des(:,i-1)-r0))^2-zd)*v + k2*w;
    r1_des(:,i) = r1_des(:,i-1) + r1dot_des*dt;
    r2_des(:,i) = r2_des(:,i-1) + r2dot_des*dt;
    % plot
    %plot(r1_des(1,i-1),r1_des(2,i-1),'go', 'MarkerFaceColor','g'); %Plots 'Robot 1'
    %plot(r2_des(1,i-1),r2_des(2,i-1),'co', 'MarkerFaceColor','c'); %Plots 'Robot 1'
    
    
    %Robot1
    % Transformation
    u1 = r1dot_des(1);
    u2 = r1dot_des(2);
    Rinv1 = [cos(theta1(1,i-1)) sin(theta1(1,i-1));-sin(theta1(1,i-1)) cos(theta1(1,i-1))];
    Lm = [1 0;0 1/l];
    vw1 = Lm*Rinv1*[u1; u2];
    v1 = vw1(1,1);
    omega1 = vw1(2,1);
    
    % Caculate velocities
    x1dot = v1*cos(theta1(1,i-1));
    y1dot = v1*sin(theta1(1,i-1));
    theta1dot = omega1;
    
    % Update motion
    x1(1,i)=x1(1,i-1)+dt*x1dot;
    y1(1,i)=y1(1,i-1)+dt*y1dot;
    theta1(1,i) = theta1(1,i-1)+dt*theta1dot;
    
    %Robot2
    % Transformation
    u1_2 = r2dot_des(1);
    u2_2 = r2dot_des(2);
    Rinv2 = [cos(theta2(1,i-1)) sin(theta2(1,i-1));-sin(theta2(1,i-1)) cos(theta2(1,i-1))];
    Lm = [1 0;0 1/l];
    vw2 = Lm*Rinv2*[u1_2; u2_2];
    v2 = vw2(1,1);
    omega2 = vw2(2,1);
    
    % Caculate velocities
    x2dot = v2*cos(theta2(1,i-1));
    y2dot = v2*sin(theta2(1,i-1));
    theta2dot = omega2;
    
    % Update motion
    x2(1,i)=x2(1,i-1)+dt*x2dot;
    y2(1,i)=y2(1,i-1)+dt*y2dot;
    theta2(1,i) = theta2(1,i-1)+dt*theta2dot;
    
    %plot
    %plot(x2(1,i-1),y2(1,i-1),'bo','MarkerFaceColor','b');
    %plot(x1(1,i-1),y1(1,i-1),'bo','MarkerFaceColor','b');
    
    
    
end
if video_flag; close(myVideo); end


