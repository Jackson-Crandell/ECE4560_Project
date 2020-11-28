clc;
%% Consts
r0 = [-2;0];
r1 = [3;1];
r2 = [4;1];
k1 = .3;
dt = 0.01;
thresh = .6;
t = 2;
% r1 = zeros(2,iterations);
% r2 = zeros(2,iterations);
R = [0, -1; 1, 0];
maxcycles = 1000;
r1dot = zeros(2,maxcycles);
r2dot = zeros(2,maxcycles);
% arrowScale = 1;
video_flag = false;

% initial positions of robots
r1(1,1) = 3;
r1(2,1) = 1;
r2(1,1) = 4;
r2(2,1) = 1;

%% prepare video
if video_flag
    figure;
    %pause;
    myVideo = VideoWriter('animation.mp4', 'MPEG-4');
    myVideo.FrameRate = 20;
    myVideo.Quality = 100;
    open(myVideo);
end
x = linspace(-5,5);
y = linspace(-5,5);
[X,Y] = meshgrid(x,y);
Z = (X+2).^2+Y.^2;
contourf(X,Y,Z,40)
%% prepare plot
D = (norm(r1-r0)+norm(r2-r0))/2;
hold on
plot(r1(1),r1(2),'go','MarkerFaceColor','g')
plot(r2(1),r2(2),'go','MarkerFaceColor','g')
plot(r0(1),r0(2),'yo','MarkerFaceColor','y')
xlim([-5 5])
ylim([-5 5])

%% main loop
while D > thresh && t < maxcycles
   %Calculate Velocities
   w = (r1(:,t-1) - r2(:,t-1)) / norm(r1(:,t-1) - r2(:,t-1));
   v = R*w;
   r1dot(:,t-1) = k1*(norm(r1(:,t-1)-r0))^2*v;
   r2dot(:,t-1) = k1*(norm(r2(:,t-1)-r0))^2*v;
   
   %Update Motion
   r1(:,t) = r1(:,t-1) + r1dot(:,t-1)*dt;
   r2(:,t) = r2(:,t-1) + r2dot(:,t-1)*dt;
   
   %Plotting Trajectory
   plot([r1(1,t-1) r1(1,t)],[r1(2,t-1) r1(2,t)],'k');
   plot([r2(1,t-1) r2(1,t)],[r2(2,t-1) r2(2,t)],'k'); 
   
   % Video recording
   if video_flag
        frame = getframe(gcf);
        writeVideo(myVideo, frame);
   end
   t = t+1;
end

%Plotting 
plot(r1(1,1),r1(2,1),'go','MarkerFaceColor','g')
plot(r2(1,1),r2(2,1),'go','MarkerFaceColor','g')
qp1 = quiver(r1(1,1),r1(2,1),r1dot(1,1)*.5,r1dot(2,1)*.5,'m','AutoScale','off');
qp2 = quiver(r2(1,1),r2(2,1),r2dot(1,1)*.5,r2dot(2,1)*.5,'m','AutoScale','off');
A = [2,30,100,200,500];
arrowScale = [.5,.5,.5,.75,1.5];
for j = 1:length(A)
    for i = A(j) 
    %    if(mod(i,150)==0)
           plot(r1(1,i-1),r1(2,i-1),'go','MarkerFaceColor','g')
           plot(r2(1,i-1),r2(2,i-1),'go','MarkerFaceColor','g')
           qp1 = quiver(r1(1,i-1),r1(2,i-1),r1dot(1,i-1)*arrowScale(j),r1dot(2,i-1)*arrowScale(j),'m','AutoScale','off');
           qp2 = quiver(r2(1,i-1),r2(2,i-1),r2dot(1,i-1)*arrowScale(j),r2dot(2,i-1)*arrowScale(j),'m','AutoScale','off');
    %    end
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
    