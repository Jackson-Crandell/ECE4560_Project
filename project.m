%% Test
a = [1, 0]';
b = [0,-2]';
S1 = .9 *[1/sqrt(30), 0;0,1];
S2 = .9 *[1,0;0,1/sqrt(15)];
A = sqrt(2)/2 * [1,-1;1,1];
g = 0.2;
[X,Y] = meshgrid(-5:0.01:5);
Z = 2 - exp(-((X.^2 + Y.^2 -a)'.*S1.*(X.^2 + Y.^2 -a))) - exp(-(X.^2 + Y.^2-b)'.*A'.*S2.*A.*(X.^2 + Y.^2-b)) + g*norm(X.^2 + Y.^2);
contour(X,Y,Z,20)

Z = zeros(length(x),length(y));
for k = 1:length(x)
    for j = 1:length(y)
        Z(k,j) = 2 - exp(-((X(k.^2 + Y.^2 -a)'.*S1.*(X.^2 + Y.^2 -a)))) - exp(-(X.^2 + Y.^2-b)'.*A'.*S2.*A.*(X.^2 + Y.^2-b)) + g*norm(X.^2 + Y.^2);
    end
end

%% Problem 1
clc;close all; clear all;
% Constants
dt=0.01; % step size or sampling frequency
tf=2; % Final time
iterations=length(0:dt:tf); %Number of iterations
x = -5:0.01:5;
y = -5:0.01:5;
[X,Y] = meshgrid(x,y);
r0 = [-2,0]'; % Source
x0 = r0(1); y0 = r0(2);
R = [0, -1; 1, 0];
k1 = 1;
r1 = zeros(2,iterations);
r2 = zeros(2,iterations);

% initial positions of robots
r1(1,1) = 3;
r1(2,1) = 1;
r2(1,1) = 4;
r2(2,1) = 1;

% Create Contour Plot
% Z = zeros(length(x),length(y));
% for k = 1:length(x)
%     for j = 1:length(y)
%         Z(k,j) = (x(k)-x0)^2 + (y(j)-y0)^2; %Scalar Field
%     end
% end
% [C,h] = contour(X,Y,Z'); hold on
% plot(r0(1),r0(2),'ro'); %Plot source in red

% Animation
for i = 2:iterations
        
    %Calculate Velocities
    w = (r1(:,i-1) - r2(:,i-1)) / norm(r1(:,i-1) - r2(:,i-1));
    v = R*w;
    r1dot = k1*(norm(r1(:,i-1)-r0))^2*v;
    r2dot = k1*(norm(r2(:,i-1)-r0))^2*v;
    
    %Plotting
%     fig=figure(1); clf;
%     % Create Contour Plot
%     Z = zeros(length(x),length(y));
%     for k = 1:length(x)
%         for j = 1:length(y)
%             Z(k,j) = (x(k)-x0)^2 + (y(j)-y0)^2; %Scalar Field
%         end
%     end
%     contour(X,Y,Z'); hold on
    if mod(iterations,50) == 0
        plot(r0(1),r0(2),'ro'); %Plot source in red
        plot(r1(1,i-1),r1(2,i-1),'go', 'MarkerFaceColor','g'); %Plots 'Robot 1'
        plot(r2(1,i-1),r2(2,i-1),'bo', 'MarkerFaceColor','b'); %Plots 'Robot 2'
        quiver(r1(1,i-1),r1(2,i-1),w(1),w(2),'k');
        quiver(r2(1,i-1),r2(2,i-1),v(1),v(2),'k');
    end

    %Update motion
    r1(:,i) = r1(:,i-1) + r1dot*dt;
    r2(:,i) = r2(:,i-1) + r2dot*dt;
    
    
end
Z = zeros(length(x),length(y));
for k = 1:length(x)
    for j = 1:length(y)
        Z(k,j) = (x(k)-x0)^2 + (y(j)-y0)^2; %Scalar Field
    end
end

C_levels = [0:5:60]; 
[C,h] = contourf(X,Y,Z',C_levels); hold on
%clabel(C,h)
plot(r0(1),r0(2),'ro'); %Plot source in red

for i = 1:iterations
    if mod(i,20) == 0
        w = (r1(:,i-1) - r2(:,i-1)) / norm(r1(:,i-1) - r2(:,i-1));
        v = R*w;
        plot(r1(1,i),r1(2,i),'go', 'MarkerFaceColor','g'); %Plots 'Robot 1'
        plot(r2(1,i),r2(2,i),'bo', 'MarkerFaceColor','b'); %Plots 'Robot 2'
        quiver(r1(1,i),r1(2,i),v(1),v(2),'k');
        quiver(r2(1,i),r2(2,i),v(1),v(2),'k');
    end
end

%% Problem 2b
clc;close all; clear all;
% Constants
dt=0.01; % step size or sampling frequency
tf=6; % Final time
iterations=length(0:dt:tf); %Number of iterations
x = -5:0.01:5;
y = -5:0.01:5;
[X,Y] = meshgrid(x,y);
r0 = [-2,0]'; % Source
x0 = r0(1); y0 = r0(2);
R = [0, -1; 1, 0];
k1 = 3;
k2 = 1;
r1 = zeros(2,iterations);
r2 = zeros(2,iterations);
zd = 2;

% initial positions of robots
r1(1,1) = 3;
r1(2,1) = 1;
r2(1,1) = 4;
r2(2,1) = 1;

% % Create Contour Plot
% Z = zeros(length(x),length(y));
% for k = 1:length(x)
%     for j = 1:length(y)
%         Z(k,j) = (x(k)-x0)^2 + (y(j)-y0)^2; %Scalar Field
%     end
% end
% contour(X,Y,Z'); hold on
plot(r0(1),r0(2),'ro'); %Plot source in red

% Animation
for i = 2:iterations
        
    %Calculate Velocities
    w = (r1(:,i-1) - r2(:,i-1)) / norm(r1(:,i-1) - r2(:,i-1));
    v = R*w;
    r1dot = k1*((norm(r1(:,i-1)-r0))^2-zd)*v + k2*w;
    r2dot = k1*((norm(r2(:,i-1)-r0))^2-zd)*v + k2*w;
    
    %Plotting
    %plot(r1(1,i-1),r1(2,i-1),'go', 'MarkerFaceColor','g'); %Plots 'Robot 1'
    %plot(r2(1,i-1),r2(2,i-1),'bo', 'MarkerFaceColor','b'); %Plots 'Robot 2'
    %quiver(r1(1,i-1),r1(2,i-1),w(1),w(2),'br');
    %quiver(r2(1,i-1),r2(2,i-1),v(1),v(2),'br');

    %Update motion
    r1(:,i) = r1(:,i-1) + r1dot*dt;
    r2(:,i) = r2(:,i-1) + r2dot*dt;
    
    
end
Z = zeros(length(x),length(y));
for k = 1:length(x)
    for j = 1:length(y)
        Z(k,j) = (x(k)-x0)^2 + (y(j)-y0)^2; %Scalar Field
    end
end

C_levels = [2,10:10:60]; 
[C,h] = contour(X,Y,Z',C_levels); hold on
clabel(C,h)

for i = 1:iterations
    plot(r1(1,i),r1(2,i),'go', 'MarkerFaceColor','g'); %Plots 'Robot 1'
    plot(r2(1,i),r2(2,i),'bo', 'MarkerFaceColor','b'); %Plots 'Robot 2'
end

%% Problem 2c
clc;close all; clear all;
% Constants
dt=0.01; % step size or sampling frequency
tf=20; % Final time
iterations=length(0:dt:tf); %Number of iterations
x = -5:0.01:5;
y = -5:0.01:5;
[X,Y] = meshgrid(x,y);
r0 = [-2,0]'; % Source
x0 = r0(1); y0 = r0(2);
R = [0, -1; 1, 0];
k1 = 3;
k2 = 0;
r1 = zeros(2,iterations);
r2 = zeros(2,iterations);
zd = 2;

% initial positions of robots
r1(1,1) = 3;
r1(2,1) = 1;
r2(1,1) = 4;
r2(2,1) = 1;

% Create Contour Plot
Z = zeros(length(x),length(y));
for k = 1:length(x)
    for j = 1:length(y)
        Z(k,j) = (x(k)-x0)^2 + (y(j)-y0)^2; %Scalar Field
    end
end
contour(X,Y,Z'); hold on
plot(r0(1),r0(2),'ro'); %Plot source in red

% Animation
for i = 2:iterations
        
    %Calculate Velocities
    w = (r1(:,i-1) - r2(:,i-1)) / norm(r1(:,i-1) - r2(:,i-1));
    v = R*w;
    r1dot = k1*((norm(r1(:,i-1)-r0))^2-zd)*v + k2*w;
    r2dot = k1*((norm(r2(:,i-1)-r0))^2-zd)*v + k2*w;
    
%     %Plotting
%     if mod(iterations,50) == 0
%         plot(r1(1,i-1),r1(2,i-1),'go', 'MarkerFaceColor','g'); %Plots 'Robot 1'
%         plot(r2(1,i-1),r2(2,i-1),'bo', 'MarkerFaceColor','b'); %Plots 'Robot 2'
%         quiver(r1(1,i-1),r1(2,i-1),w(1),w(2),'k');
%         quiver(r2(1,i-1),r2(2,i-1),v(1),v(2),'k');
%     end

    %Update motion
    r1(:,i) = r1(:,i-1) + r1dot*dt;
    r2(:,i) = r2(:,i-1) + r2dot*dt;
    
    
end

% for i = 1:iterations
%     plot(r1(1,i),r1(2,i),'go', 'MarkerFaceColor','g'); %Plots 'Robot 1'
%     plot(r2(1,i),r2(2,i),'bo', 'MarkerFaceColor','b'); %Plots 'Robot 2'
% end

%% Problem 2e
clc;close all; clear all;
% Constants
dt=0.01; % step size or sampling frequency
tf=20; % Final time
iterations=length(0:dt:tf); %Number of iterations
%r0 = [-2,0]'; % Source
%x0 = r0(1); y0 = r0(2);
R = [0, 1; -1, 0];
k1 = 10;
k2 = 1;
r1 = zeros(2,iterations);
r2 = zeros(2,iterations);
zd = 2;

% initial positions of robots
r1(1,1) = 3;
r1(2,1) = 1;
r2(1,1) = 4;
r2(2,1) = 1;

a = [1, 0]';
b = [0,-2]';
S1 = .9 *[1/sqrt(30), 0;0,1];
S2 = .9 *[1,0;0,1/sqrt(15)];
A = sqrt(2)/2 * [1,-1;1,1];
g = 0.2;

% Create Contour Plot
x = -5:0.01:5;
y = -5:0.01:5;
[X,Y] = meshgrid(x,y);
Z = zeros(length(x),length(y));
for k = 1:length(x)
    for j = 1:length(y)
        Z(k,j) = 2 - exp(-((x(k)^2 + y(j)^2 -a)'*S1*(x(k)^2 + y(j)^2-a))) - exp(-(x(k)^2 + y(j)^2-b)'*A'*S2*A*(x(k)^2 + y(j)^2-b)) + g*norm(x(k)^2 + y(j)^2);
    end
end
contour(X,Y,Z); hold on
%plot(r0(1),r0(2),'ro'); %Plot source in red

% Animation
for i = 2:iterations
        
    %Calculate Velocities
    w = (r1(:,i-1) - r2(:,i-1)) / norm(r1(:,i-1) - r2(:,i-1));
    v = R*w;
    r1dot = k1*(( 2 - exp(-((r1(:,i-1)) -a)'*S1*((r1(:,i-1))-a)) - exp(-((r1(:,i-1))-b)'*A'*S2*A*((r1(:,i-1))-b)) + g*norm(r1(:,i-1))) - zd)*v + k2*w;
    r2dot = k1*(( 2 - exp(-((r2(:,i-1)) -a)'*S1*((r2(:,i-1))-a)) - exp(-((r2(:,i-1))-b)'*A'*S2*A*((r2(:,i-1))-b)) + g*norm(r2(:,i-1))) - zd)*v + k2*w;
    
    %Plotting
    plot(r1(1,i-1),r1(2,i-1),'go', 'MarkerFaceColor','g'); %Plots 'Robot 1'
    plot(r2(1,i-1),r2(2,i-1),'bo', 'MarkerFaceColor','b'); %Plots 'Robot 2'
    %quiver(r1(1,i-1),r1(2,i-1),w(1),w(2),'br');
    %quiver(r2(1,i-1),r2(2,i-1),v(1),v(2),'br');

    %Update motion
    r1(:,i) = r1(:,i-1) + r1dot*dt;
    r2(:,i) = r2(:,i-1) + r2dot*dt;
    
    
end

% for i = 1:iterations
%     plot(r1(1,i),r1(2,i),'go', 'MarkerFaceColor','g'); %Plots 'Robot 1'
%     %plot(r2(1,i),r2(2,i),'bo', 'MarkerFaceColor','b'); %Plots 'Robot 2'
% end

%% Problem 3 (Part 1)
clc;close all; clear all;
% Constants
p = -5:0.01:5;
c = -5:0.01:5;
[X,Y] = meshgrid(p,c);
l=0.07; 
dt=0.001; % step size or sampling frequency
tf=2; % Final time
iterations=length(0:dt:tf); %Number of iterations
R = [0, -1; 1, 0];
k1 = 1;
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
theta2(1,1)=theta2_0;

for i=2:iterations

    % Plotting
%     fig=figure(1); clf;
%     pose=[x(1,i-1);y1(1,i-1);theta1(1,i-1)];
%     plot_robot(pose,fig) %Plot the robot location
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
    r1dot_des = k1*(norm(r1_des(:,i-1)-r0))^2*v;
    r1_des(:,i) = r1_des(:,i-1) + r1dot_des*dt;
    r2dot_des = k1*(norm(r2_des(:,i-1)-r0))^2*v;
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
    x1dot = v1*cos(theta1(1,i-1));
    y1dot = v1*sin(theta1(1,i-1));
    theta1dot = omega1;
    
    % Update motion
    x1(1,i)=x1(1,i-1)+dt*x1dot;
    y1(1,i)=y1(1,i-1)+dt*y1dot;
    theta1(1,i) = theta1(1,i-1)+dt*theta1dot;
    
    %Robot1
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


end

Z = zeros(length(p),length(c));
for k = 1:length(p)
    for j = 1:length(c)
        Z(k,j) = (p(k)-r0(1))^2 + (c(j)-r0(2))^2; %Scalar Field
    end
end
contour(X,Y,Z'); hold on
plot(r0(1),r0(2),'ro'); %Plot source in red

for i = 2:iterations
    plot(r1_des(1,i-1),r1_des(2,i-1),'go', 'MarkerFaceColor','g'); %Plots 'Robot 1'
    plot(x1(1,i-1),y1(1,i-1),'bo','MarkerFaceColor','b');
    plot(r2_des(1,i-1),r2_des(2,i-1),'co', 'MarkerFaceColor','c'); %Plots 'Robot 1'
    plot(x2(1,i-1),y2(1,i-1),'bo','MarkerFaceColor','b');
   
end

%% Problem 3 (Part 2b)
clc;close all; clear all;
% Constants
p = -5:0.01:5;
c = -5:0.01:5;
[X,Y] = meshgrid(p,c);
l=0.07; 
dt=0.001; % step size or sampling frequency
tf=6; % Final time
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
theta2(1,1)=theta2_0;

for i=2:iterations

    % Plotting
%     fig=figure(1); clf;
%     pose=[x(1,i-1);y1(1,i-1);theta1(1,i-1)];
%     plot_robot(pose,fig) %Plot the robot location
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
    
    %Robot1
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


end

Z = zeros(length(p),length(c));
for k = 1:length(p)
    for j = 1:length(c)
        Z(k,j) = (p(k)-r0(1))^2 + (c(j)-r0(2))^2; %Scalar Field
    end
end
contour(X,Y,Z'); hold on
plot(r0(1),r0(2),'ro'); %Plot source in red

for i = 2:iterations
    plot(r1_des(1,i-1),r1_des(2,i-1),'go', 'MarkerFaceColor','g'); %Plots 'Robot 1'
    plot(x1(1,i-1),y1(1,i-1),'bo','MarkerFaceColor','b');
    plot(r2_des(1,i-1),r2_des(2,i-1),'co', 'MarkerFaceColor','c'); %Plots 'Robot 1'
    plot(x2(1,i-1),y2(1,i-1),'bo','MarkerFaceColor','b');
   
end

%% Problem 3 (Part 2e)
clc;close all; clear all;
% Constants
p = -5:0.01:5;
c = -5:0.01:5;
[X,Y] = meshgrid(p,c);
l=0.07; 
dt=0.001; % step size or sampling frequency
tf=20; % Final time
iterations=length(0:dt:tf); %Number of iterations
R = [0, -1; 1, 0];
k1 = 3;
k2 = 1;
zd = 2;
a = [1, 0]';
b = [0,-2]';
S1 = .9 *[1/sqrt(30), 0;0,1];
S2 = .9 *[1,0;0,1/sqrt(15)];
A = sqrt(2)/2 * [1,-1;1,1];
g = 0.2;

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
theta2(1,1)=theta2_0;

for i=2:iterations

    % Plotting
%     fig=figure(1); clf;
%     pose=[x(1,i-1);y1(1,i-1);theta1(1,i-1)];
%     plot_robot(pose,fig) %Plot the robot location
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
    r1dot_des = k1*(( 2 - exp(-((r1_des(:,i-1)) -a)'*S1*((r1_des(:,i-1))-a)) - exp(-((r1_des(:,i-1))-b)'*A'*S2*A*((r1_des(:,i-1))-b)) + g*norm(r1_des(:,i-1))) - zd)*v + k2*w;
    r2dot_des = k1*(( 2 - exp(-((r2_des(:,i-1)) -a)'*S1*((r2_des(:,i-1))-a)) - exp(-((r2_des(:,i-1))-b)'*A'*S2*A*((r2_des(:,i-1))-b)) + g*norm(r2_des(:,i-1))) - zd)*v + k2*w;
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
    x1dot = v1*cos(theta1(1,i-1));
    y1dot = v1*sin(theta1(1,i-1));
    theta1dot = omega1;
    
    % Update motion
    x1(1,i)=x1(1,i-1)+dt*x1dot;
    y1(1,i)=y1(1,i-1)+dt*y1dot;
    theta1(1,i) = theta1(1,i-1)+dt*theta1dot;
    
    %Robot1
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


end

Z = zeros(length(p),length(c));
for k = 1:length(p)
    for j = 1:length(c)
        Z(k,j) = (p(k)-r0(1))^2 + (c(j)-r0(2))^2; %Scalar Field
    end
end
contour(X,Y,Z'); hold on
plot(r0(1),r0(2),'ro'); %Plot source in red

for i = 2:iterations
    plot(r1_des(1,i-1),r1_des(2,i-1),'go', 'MarkerFaceColor','g'); %Plots 'Robot 1'
    plot(x1(1,i-1),y1(1,i-1),'bo','MarkerFaceColor','b');
    plot(r2_des(1,i-1),r2_des(2,i-1),'co', 'MarkerFaceColor','c'); %Plots 'Robot 1'
    plot(x2(1,i-1),y2(1,i-1),'bo','MarkerFaceColor','b');
   
end

