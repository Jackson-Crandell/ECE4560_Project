%% Problem 3 (Part 2b)
clc;close all; clear all;
% Constants
p = -5:0.01:5;
c = -5:0.01:5;
[X,Y] = meshgrid(p,c);
l=0.07; 
dt=0.001; % step size or sampling frequency
tf=9; % Final time
iterations=length(0:dt:tf); %Number of iterations
R = [0, -1; 1, 0];
k1 = 3;
k2 = 1;
zd = 2;
thresh = .52;

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
x1 = zeros(1,iterations-1);
y1 = zeros(1,iterations-1);
x1dot = zeros(1,iterations-1);
y1dot = zeros(1,iterations-1);
theta1 = zeros(1,iterations);
x1(1,1) = x1_0;
y1(1,1) = y1_0;
r0 = [-2,0]';
theta1(1,1)=theta1_0;

x2_0 = r2_des(1,1);
y2_0 = r2_des(2,1); %-0.1; ??
theta2_0 = pi/2;
x2 = zeros(1,iterations-1);
y2 = zeros(1,iterations-1);
x2dot = zeros(1,iterations-1);
y2dot = zeros(1,iterations-1);
theta2 = zeros(1,iterations);
x2(1,1) = x2_0;
y2(1,1) = y2_0;
r0 = [-2,0]';
r1 = [3 1];
r2 = [4 1];
theta2(1,1)=theta2_0;

D = (norm(r1-r0)+norm(r2-r0))/2;

% prepare video
video_flag = 0;
if video_flag
    fig = figure(1);
    %pause;
    myVideo = VideoWriter('animation3_2b.mp4', 'MPEG-4');
    myVideo.FrameRate = 100;
    myVideo.Quality = 75;
    open(myVideo);
end

i = 2;
animate = false;
while D > thresh && i < iterations

    % Plotting
    if animate
        if(i<30 || (i<45 && mod(i,2)==0) || mod(i,30)==0)
            fig=figure(1); 
            clf;
            pose=[x1(1,i-1);y1(1,i-1);theta1(1,i-1)];
            pose2 = [x2(1,i-1);y2(1,i-1);theta2(1,i-1)];
            [M,d] = contourf(X,Y,Z'); hold on
            [C,h] = contour(X, Y, Z', [2 2], 'k', 'LineWidth', 3); 
            clabel(C,h,2,'FontWeight','bold','FontSize',15) % Label 2 contour
            plot(x1(1,1:i-1),y1(1,1:i-1),'r', 'LineWidth', 1.5)
            plot(x2(1,1:i-1),y2(1,1:i-1),'r', 'LineWidth', 1.5)
            plot(r0(1),r0(2),'ro'); %Plot source in red
            plot_robot(pose,fig) %Plot the robot location
            plot_robot(pose2,fig) %Plot the robot location
            plot(r1(1),r1(2),'go')
            plot(r2(1),r2(2),'go')
            if video_flag
                frame = getframe(gcf);
                writeVideo(myVideo, frame);
            end
        end
    end
 
    % Desired Trajectory
    w = (r1_des(:,i-1) - r2_des(:,i-1)) / norm(r1_des(:,i-1) - r2_des(:,i-1));
    v = R*w;
    r1dot_des = k1*((norm(r1_des(:,i-1)-r0))^2-zd)*v + k2*w;
    r2dot_des = k1*((norm(r2_des(:,i-1)-r0))^2-zd)*v + k2*w;
    r1_des(:,i) = r1_des(:,i-1) + r1dot_des*dt;
    r2_des(:,i) = r2_des(:,i-1) + r2dot_des*dt;
     
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
    x1dot(1,i) = v1*cos(theta1(1,i-1));
    y1dot(1,i) = v1*sin(theta1(1,i-1));
    theta1dot = omega1;
    
    % Update motion
    x1(1,i)=x1(1,i-1)+dt*x1dot(1,i-1);
    y1(1,i)=y1(1,i-1)+dt*y1dot(1,i-1);
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
    x2dot(1,i) = v2*cos(theta2(1,i-1));
    y2dot(1,i) = v2*sin(theta2(1,i-1));
    theta2dot = omega2;
    
    % Update motion
    x2(1,i)=x2(1,i-1)+dt*x2dot(1,i-1);
    y2(1,i)=y2(1,i-1)+dt*y2dot(1,i-1);
    theta2(1,i) = theta2(1,i-1)+dt*theta2dot;
    
    %Update D
    D = (norm(r1-r0)+norm(r2-r0))/2;
    
    i = i + 1;
    
end

%% Plotting 
plots = true;
if plots 
    xMIN = -5;
    xMAX = 5;
    yMIN = -5;
    yMAX = 5;

    xlim([xMIN xMAX])
    ylim([yMIN yMAX])
    %Contour Plot
    Z = zeros(length(p),length(c));
    for k = 1:length(p)
        for j = 1:length(c)
            Z(k,j) = (p(k)-r0(1))^2 + (c(j)-r0(2))^2; %Scalar Field
        end
    end
    [M,d] = contourf(X,Y,Z'); hold on
    [C,h] = contour(X, Y, Z', [2 2], 'k', 'LineWidth', 3); 
    clabel(C,h,2,'FontWeight','bold','FontSize',15) % Label 2 contour
    plot(r0(1),r0(2),'ro'); %Plot source in red
    
    fig=figure(1);
    A = [2,50,100,2000,4000,6000];
    arrowScale = [.25,.1,.1,.5,.5,.5];
    drawArrows = false;
    %Plot Trajectory
    plot(x1(1,1:end),y1(1,1:end),'r', 'LineWidth', 1.5)
    plot(x2(1,1:end),y2(1,1:end),'r', 'LineWidth', 1.5)
    for j = 1:length(A)
        for i = A(j) 
            pose=[x1(1,i-1);y1(1,i-1);theta1(1,i-1)];
            pose2 = [x2(1,i-1);y2(1,i-1);theta2(1,i-1)];
            plot_robot(pose,fig) %Plot the robot location
            plot_robot(pose2,fig) %Plot the robot location
            if drawArrows
                qp1 = quiver(x1(1,i-1),y1(1,i-1),x1dot(1,i-1)*arrowScale(j),y1dot(1,i-1)*arrowScale(j),'m','AutoScale','off');
                qp1.LineWidth = 1.5;
                qp1.MaxHeadSize = .5;
                qp2 = quiver(x2(1,i-1),y2(1,i-1),x2dot(1,i-1)*arrowScale(j),y2dot(1,i-1)*arrowScale(j),'m','AutoScale','off');
                qp2.LineWidth = 1.5;
                qp2.MaxHeadSize = .5;
            end
        end
    end
    
    pose=[x1(1,end);y1(1,end);theta1(1,end)];
    pose2 = [x2(1,end);y2(1,end);theta2(1,end)];
    plot_robot(pose,fig) %Plot the robot final location
    plot_robot(pose2,fig) 
end



if video_flag; close(myVideo); end


