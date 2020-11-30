clc; clear all;
%Constants
thresh = .52;
i = 2;
maxcycles = 1200;
arrowScale = 40;
video_flag = false;
dt=0.01; % step size or sampling frequency
x = -5:0.01:5;
y = -5:0.01:5;
[X,Y] = meshgrid(x,y);
r0 = [-2,0]'; % Source
x0 = r0(1); y0 = r0(2);
R = [0, -1; 1, 0];
k1 = 3;
k2 = 1;
zd = 2;
r1dot = zeros(2,maxcycles);
r2dot = zeros(2,maxcycles);

% initial positions of robots
r1(1,1) = 3;
r1(2,1) = 1;
r2(1,1) = 4;
r2(2,1) = 1;

%Prepare video
if video_flag
    figure;
    %pause;
    myVideo = VideoWriter('animation2.mp4', 'MPEG-4');
    myVideo.FrameRate = 100;
    myVideo.Quality = 100;
    open(myVideo);
end

%Prepare plot
hold on
plot(r1(1),r1(2),'go')
plot(r2(1),r2(2),'go')
plot(r0(1),r0(2),'bo')
xlim([-5 5])
ylim([-5 5])

x = linspace(-5,5);
y = linspace(-5,5);
[X,Y] = meshgrid(x,y);
Z = (X+2).^2+Y.^2;
C_levels = [0,2,10:10:80]; 
[M,d] = contourf(X,Y,Z,C_levels); hold on
[C,h] = contour(X, Y, Z, [2 2], 'k', 'LineWidth', 3); 
clabel(C,h,2,'FontWeight','bold','FontSize',15) % Label 2 contour

w = (r1(:,i-1) - r2(:,i-1)) / norm(r1(:,i-1) - r2(:,i-1));


%Main loop
while i < maxcycles
        
    %Calculate Velocities
    w = (r1(:,i-1) - r2(:,i-1)) / norm(r1(:,i-1) - r2(:,i-1));
    v = R*w;
    r1dot(:,i-1) = k1*((norm(r1(:,i-1)-r0))^2-zd)*v + k2*w;
    r2dot(:,i-1) = k1*((norm(r2(:,i-1)-r0))^2-zd)*v + k2*w;
    
    %Update motion
    r1(:,i) = r1(:,i-1) + r1dot(:,i-1)*dt;
    r2(:,i) = r2(:,i-1) + r2dot(:,i-1)*dt;
    
    %Plotting Trajectory
    plot([r1(1,i-1) r1(1,i)],[r1(2,i-1) r1(2,i)],'y','LineWidth',1.5);
    plot([r2(1,i-1) r2(1,i)],[r2(2,i-1) r2(2,i)],'y','LineWidth',1.5); 
    
    i = i + 1;
    
    % Video recording
    if video_flag
        frame = getframe(gcf);
        writeVideo(myVideo, frame);
    end
    
end
   
%Plotting 
plot(r1(1,1),r1(2,1),'go','MarkerFaceColor','g')
plot(r2(1,1),r2(2,1),'go','MarkerFaceColor','g')
A = [2,10,50,200,500];
arrowScale = [.05,.05,.5,.5,.5];
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

% Video
if video_flag
        frame = getframe(gcf);
        writeVideo(myVideo, frame);
end
if video_flag; close(myVideo); end
hold off
    
