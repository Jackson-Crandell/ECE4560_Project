clc; clear all;
%Constants
r0 = [-2;0];
thresh = .52;
i = 2;
maxcycles = 2000;
arrowScale = 40;
video_flag = false;

dt=0.01; % step size or sampling frequency
R = [0, 1; -1, 0];
k1 = 10;
k2 = 1;
zd = 2;
a = [1;0];
b = [0;-2];
S1 = 0.9*[(1/sqrt(30)) 0;0 1];
S2 = 0.9*[1 0;0 (1/sqrt(15))];
A = (sqrt(2)/2)*[1 -1;1 1];
r1dot = zeros(2,maxcycles);
r2dot = zeros(2,maxcycles);

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

%Prepare video
if video_flag
    figure;
    %pause;
    myVideo = VideoWriter('animation3.mp4', 'MPEG-4');
    myVideo.FrameRate = 100;
    myVideo.Quality = 100;
    open(myVideo);
end

%Prepare plot
D = (norm(r1-r0)+norm(r2-r0))/2;

% Create Contour Plot
xMIN = -5;
xMAX = 5;
yMIN = -5;
yMAX = 5;

xlim([xMIN xMAX])
ylim([yMIN yMAX])

x = linspace(xMIN,xMAX);
y = linspace(yMIN,yMAX);
[X,Y] = meshgrid(x,y);
Z = zeros(length(y),length(x));
for i = 1:length(x)
    for j = 1:length(y)
        Z(j,i) = 2-exp(-([X(1,i);Y(j,1)]-a)'*S1*([X(1,i);Y(j,1)]-a))-exp(-([X(1,i);Y(j,1)]-b)'*A'*S2*A*([X(1,i);Y(j,1)]-b))+g*norm([X(1,i);Y(j,1)]);
    end
end
C_levels = [0,2,1:.2:3.5]; 
[M,d] = contourf(X,Y,Z,C_levels); hold on
[C,h] = contour(X, Y, Z, [2 2], 'k', 'LineWidth', 3); 
clabel(C,h,2,'FontWeight','bold','FontSize',15) % Label 2 contour

%Main loop
i = 2;
while D > thresh && i < maxcycles
    
    %Calculate Velocities
    w = (r1(:,i-1) - r2(:,i-1)) / norm(r1(:,i-1) - r2(:,i-1));
    v = R*w;
    r1dot(:,i-1) = k1*(( 2 - exp(-((r1(:,i-1)) -a)'*S1*((r1(:,i-1))-a)) - exp(-((r1(:,i-1))-b)'*A'*S2*A*((r1(:,i-1))-b)) + g*norm(r1(:,i-1))) - zd)*v + k2*w;
    r2dot(:,i-1) = k1*(( 2 - exp(-((r2(:,i-1)) -a)'*S1*((r2(:,i-1))-a)) - exp(-((r2(:,i-1))-b)'*A'*S2*A*((r2(:,i-1))-b)) + g*norm(r2(:,i-1))) - zd)*v + k2*w;

    %Update motion
    r1(:,i) = r1(:,i-1) + r1dot(:,i-1)*dt;
    r2(:,i) = r2(:,i-1) + r2dot(:,i-1)*dt;
    
    %Update D
    D = (norm(r1-r0)+norm(r2-r0))/2;
    
    %Plotting Trajectory
    plot([r1(1,i-1) r1(1,i)],[r1(2,i-1) r1(2,i)],'r','LineWidth',1.5);
    plot([r2(1,i-1) r2(1,i)],[r2(2,i-1) r2(2,i)],'r','LineWidth',1.5); 
    
    % Video recording
    if video_flag
        frame = getframe(gcf);
        writeVideo(myVideo, frame);
    end
    
    i = i + 1;
end

%Plotting 
plot(r1(1,1),r1(2,1),'go','MarkerFaceColor','g')
plot(r2(1,1),r2(2,1),'go','MarkerFaceColor','g')
A = [2,50,100,200,500,800];
arrowScale = [.25,.25,.25,.5,.5,.5];
for j = 1:length(A)
    for i = A(j) 
           plot(r1(1,i-1),r1(2,i-1),'go','MarkerFaceColor','g')
           plot(r2(1,i-1),r2(2,i-1),'go','MarkerFaceColor','g')
           qp1 = quiver(r1(1,i-1),r1(2,i-1),r1dot(1,i-1)*arrowScale(j),r1dot(2,i-1)*arrowScale(j),'m','AutoScale','off');
           qp1.LineWidth = 1.5;
           qp1.MaxHeadSize = .5;
           qp2 = quiver(r2(1,i-1),r2(2,i-1),r2dot(1,i-1)*arrowScale(j),r2dot(2,i-1)*arrowScale(j),'m','AutoScale','off');
           qp2.LineWidth = 1.5;
           qp2.MaxHeadSize = .5;
    end
end

plot(r1(1,end),r1(2,end),'ro','MarkerFaceColor','r')
plot(r2(1,end),r2(2,end),'ro','MarkerFaceColor','r')

if video_flag
        frame = getframe(gcf);
        writeVideo(myVideo, frame);
end
if video_flag; close(myVideo); end
hold off
    