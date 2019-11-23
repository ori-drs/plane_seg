function [pos,lookdir] = find_sensor_position(p)

p = sortrows(p,2);
starts = [1;find(diff(p(:,2))>0)+1];
ends = [starts(2:end)-1;size(p,1)];
all_planes = zeros(numel(starts),4);
all_s = zeros(numel(starts),3);
e = [];
for i = 1:numel(starts)
    pts_orig = p(starts(i):ends(i),end-2:end);
    avg = mean(pts_orig);
    pts_cur = pts_orig-repmat(avg,[size(pts_orig,1),1]);
    [~,s,v] = svd(pts_cur,0);
    plane = [v(:,end)',-dot(v(:,end),avg)];
    e = [e;pts_orig*plane(1:3)' + plane(4)];
    all_planes(i,:) = plane;
    all_s(i,:) = diag(s);
end
[~,s,v] = svd(all_planes(:,1:3),0);
lookdir = v(:,end);

prob.pts = p(:,end-2:end);
prob.r = p(:,5);

x_init = [0;0;0];
opts = optimset('display','iter');
x = lsqnonlin(@error_func,x_init,[],[],opts,prob);
pos = x(:);

rays = prob.pts-repmat(pos(:)',[size(prob.pts,1),1]);
dots = rays*lookdir(:);
if (sum(dots>0)/numel(dots) < 0.5)
    lookdir = -lookdir;
end

fprintf('pos: %f %f %f\n', pos);
fprintf('look dir: %f %f %f\n', lookdir);


function e = error_func(x,prob)
rays = [prob.pts(:,1)-x(1),prob.pts(:,2)-x(2),prob.pts(:,3)-x(3)];
radii = sqrt(sum(rays.^2,2));
e = radii-prob.r;
