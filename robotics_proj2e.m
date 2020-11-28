clc;
%% Consts
r0 = [-2;0];
r1 = [3;1];
r2 = [4;1];
zd = 2;
k1 = 0.04;
k2 = 0.02;
thresh = .52;
t = 0;
maxcycles = 500;
arrowScale = 40;
video_flag = true;

%% prepare video
if video_flag
    figure;
    %pause;
    myVideo = VideoWriter('animation3.mp4', 'MPEG-4');
    myVideo.FrameRate = 100;
    myVideo.Quality = 100;
    open(myVideo);
end
%% prepare plot
D = (norm(r1-r0)+norm(r2-r0))/2;
hold on

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
        Z(j,i) = 2-exp(-([X(1,i);Y(j,1)]-a)'*S1*([X(1,i);Y(j,1)]-a))-exp(-([X(1,i);Y(j,1)]-b)'*A'*S2*A*([X(1,i);Y(j,1)]-b))+norm([X(1,i);Y(j,1)]);
    end
end
contourf(X,Y,Z,100)

plot(r1(1),r1(2),'g.','MarkerSize',30)
plot(r2(1),r2(2),'g.','MarkerSize',30)
%plot(r0(1),r0(2),'b.','MarkerSize',30)

% add consts
a = [1;0];
b = [0;-2];
S1 = 0.9*[(1/sqrt(30)) 0;0 1];
S2 = 0.9*[1 0;0 (1/sqrt(15))];
A = (sqrt(2)/2)*[1 -1;1 1];



%% main loop
while D>thresh && t < maxcycles
   %update positions
   r1old = r1;
   r2old = r2;
   w = (r1-r2)/norm(r1-r2);
   v = flip(w).*[-1;1];
   c = k1*v;
   % calc Zs
   z1 = 2-exp(-(r1-a)'*S1*(r1-a))-exp(-(r1-b)'*A'*S2*A*(r1-b))+norm(r1);
   z2 = 2-exp(-(r2-a)'*S1*(r2-a))-exp(-(r2-b)'*A'*S2*A*(r2-b))+norm(r2);
   % add them with newx = oldx + -dy    and newy = oldy + dx
   d1 = k1*(z1-2)*v+k2*w;%flip(r1p).*[-1;1];
   d2 = k1*(z2-2)*v+k2*w;%flip(r2p).*[-1;1];
   r1 = r1+d1;
   r2 = r2+d2;
   %update D
   D = (norm(r1-r0)+norm(r2-r0))/2;
   %plot
   plot([r1old(1) r1(1)],[r1old(2) r1(2)],'k');
   plot([r2old(1) r2(1)],[r2old(2) r2(2)],'k'); 
   if(mod(t,30)==0)
       quiver(r1old(1),r1old(2),d1(1)*arrowScale,d1(2)*arrowScale);
       quiver(r2old(1),r2old(2),d2(1)*arrowScale,d2(2)*arrowScale);
   end
   % Video recording
   if video_flag
        frame = getframe(gcf);
        writeVideo(myVideo, frame);
   end
   t = t+1;
end
plot(r1(1),r1(2),'r.','MarkerSize',30)
plot(r2(1),r2(2),'r.','MarkerSize',30)
if video_flag
        frame = getframe(gcf);
        writeVideo(myVideo, frame);
end
if video_flag; close(myVideo); end
hold off
    