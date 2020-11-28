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
video_flag = false;

%% prepare video
if video_flag
    figure;
    %pause;
    myVideo = VideoWriter('animation2.mp4', 'MPEG-4');
    myVideo.FrameRate = 100;
    myVideo.Quality = 100;
    open(myVideo);
end
%% prepare plot
D = (norm(r1-r0)+norm(r2-r0))/2;
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
contourf(X,Y,Z,40)


%% main loop
while D>thresh && t < maxcycles
   %update positions
   r1old = r1;
   r2old = r2;
   w = (r1-r2)/norm(r1-r2);
   v = flip(w).*[-1;1];
   c = k1*v;
   % calculate derivatives
   r1p = c*norm(r1-r0)^2;
   r2p = c*norm(r2-r0)^2;
   % add them with newx = oldx + -dy    and newy = oldy + dx
   d1 = k1*(norm(r1-r0)^2-2)*v+k2*w;%flip(r1p).*[-1;1];
   d2 = k1*(norm(r2-r0)^2-2)*v+k2*w;%flip(r2p).*[-1;1];
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
plot(r1(1),r1(2),'ro')
plot(r2(1),r2(2),'ro')
if video_flag
        frame = getframe(gcf);
        writeVideo(myVideo, frame);
end
if video_flag; close(myVideo); end
hold off
    
