%% load data
%p = load('/home/antone/data/tilted-steps.csv');
p = load('/home/antone/data/2015-03-11_testbed/terrain_close.csv');

%% find sensor position
[sensor_pos,look_dir] = find_sensor_position(p);
pts = p(:,end-2:end);
look_pt = sensor_pos + look_dir*3;
figure, plot3k(pts);
hold on
myplot3(sensor_pos(:)','r.','markersize',30);
plot3([sensor_pos(1);look_pt(1)],[sensor_pos(2);look_pt(2)],[sensor_pos(3);look_pt(3)],'r-');
hold off;
axis equal; view3d on


%%
cloud = loadpcd('/home/antone/temp/cloud.pcd')';
normals = loadpcd('/home/antone/temp/normals.pcd')';
cloud = cloud(:,1:3);
normals = normals(:,1:3);

mmm = [cloud,cloud+0.1*normals];
mmm = mmm(1:10:end,:);
figure, plot3(mmm(:,[1,4])',mmm(:,[2,5])',mmm(:,[3,6])','r-');
hold on
myplot3(mmm(:,1:3),'b.');
hold off;
axis equal
view3d on

%% kmeans
planes = [normals,-sum(normals.*cloud,2)];
idx = kmeans(planes,30);
figure, plot3k(cloud,'ColorData',idx); axis equal; view3d on

figure
hold on
u = unique(idx);
for i = 1:numel(u)
    p = cloud(idx==u(i),:);
    myplot3(p,'.','color',rand(1,3));
end
hold off
axis equal; view3d on

%% plot raw data
foo = planes(:,[1,2,4]);
figure, myplot3(foo,'r.');
grid on
view3d on

%% plot labeled points
dat = load('/home/antone/temp/labeled_points.txt');
u = unique(dat(:,4));
figure
hold on
for i = 1:numel(u)
    if (u(i)==0)
        col = [0,0,0];
    else
        col = rand(1,3);
    end
    p = dat(dat(:,4)==u(i),1:3);
    myplot3(p,'.','color',col);
end
hold off
axis equal
view3d on

%% depth map processing
d = load('/home/antone/temp/depth_image.txt'); d(isinf(d)) = 0;
d_orig = d;
for iter = 1:5
    bad_mask = d==0;
    d_filt = medfilt2(d,[5,5]);
    d(bad_mask) = d_filt(bad_mask);
end
figure, plothot(cat(2,d_orig,d));

%% range image
ranges = load('/home/antone/temp/ranges.txt');
fov = 120*pi/180;
res = fov/size(ranges,2);
[x,y] = meshgrid(0:size(ranges,2)-1, 0:size(ranges,1)-1);
x = (x-size(ranges,2)/2)*res;
y = (y-size(ranges,1)/2)*res;

pts = [ranges(:).*sin(x(:)).*cos(y(:)), ranges(:).*sin(y(:)), ranges(:).*cos(x(:)).*cos(y(:))];
pts(ranges==0,:) = [];
figure, plot3k(pts); axis equal; view3d on


%% process depths
d = load('/home/antone/temp/depth_image.txt'); d(isinf(d)) = 0;
cloud = loadpcd('/home/antone/temp/organized_points.pcd');
normals = loadpcd('/home/antone/temp/organized_normals.pcd');
figure, plothot(sqrt(normals(:,:,4)));

[gx,gy] = gradient(d);
gmag = hypot(gx,gy);
figure, plothot(gmag,[0,0.05]);

msk = d>0;
x = cloud(:,:,1);
y = cloud(:,:,2);
z = cloud(:,:,3);
pts = [x(msk),y(msk),z(msk)];
nx = normals(:,:,1);
ny = normals(:,:,2);
nz = normals(:,:,3);
norms = [nx(msk),ny(msk),nz(msk)];

planes = zeros(size(normals,1),size(normals,2),4);
planes(:,:,1:3) = normals(:,:,1:3);
planes(:,:,4) = -sum(normals(:,:,1:3).*cloud(:,:,1:3),3);

figure, plothot(planes(:,:,3));

bb = planes(:,:,4);
bb = bb-min(bb(:));
bb = bb/max(bb(:));

rr = (planes(:,:,1)+1)/2;
gg = (planes(:,:,2)+1)/2;
rgb = cat(3, rr,gg,bb);
figure, imshow(rgb);

[gx_shift,gy_shift] = gradient(planes(:,:,4));


% check if each point is different from any of its neighbors
%   each point has a plane equation
%   what is the right distance metric?
%     could be offset>x OR normal angle

if (false)
    edge_mask = false(size(planes,1),size(planes,2));
    for i = 2:size(planes,1)-1
        for j = 2:size(planes,2)-1
            if (d(i,j)==0)
                continue
            end
            p0 = squeeze(cloud(i,j,:));
            plane0 = squeeze(planes(i,j,:));
            pts = [squeeze(cloud(i,j-1,1:3))'
                squeeze(cloud(i,j+1,1:3))';
                squeeze(cloud(i-1,j,1:3))';
                squeeze(cloud(i+1,j,1:3))'];
            dists = pts*plane0(1:3) + plane0(4);
            if (any(abs(dists)>0.01))
                edge_mask(i,j) = true;
            end
        end
    end
    figure, plothot(edge_mask);
end


% mmm = [pts,pts+0.1*norms];
% figure
% hold on
% plot3(mmm(1:10:end,[1,4])',mmm(1:10:end,[2,5])',mmm(1:10:end,[3,6])','r-');
% myplot3(mmm(1:10:end,1:3),'b.');
% hold off;
% axis equal
% view3d on


%% look at labels
lab = load('/home/antone/temp/labels.txt');
cloud = loadpcd('/home/antone/temp/organized_points.pcd');
props = regionprops(lab,'Area');
areas = [props.Area];
[areas,sort_ind] = sort(areas,'descend');
value_map(sort_ind) = 1:numel(sort_ind);
lab(lab>0) = value_map(lab(lab>0));
%lab(lab>30) = 0;
figure, plothot(lab);


x = cloud(:,:,1);
y = cloud(:,:,2);
z = cloud(:,:,3);
figure
hold on
for i = 1:30
    ind = lab==i;
    myplot3([x(ind),y(ind),z(ind)],'.','color',rand(1,3));
end
hold off
axis equal
view3d on


%% per point normals and labels
basedir = '/home/antone/drc/software/perception/plane-seg';
cloud_full = loadpcd([basedir,'/cloud_full.pcd'])';
cloud = loadpcd([basedir,'/cloud.pcd'])';
normals = loadpcd([basedir,'/robust_normals.pcd'])';
labels = load([basedir,'/labels.txt']);

u = unique(labels);
figure, hold on
%myplot3(cloud_full(:,1:3),'r.');
for i = 1:numel(u)
    if (u(i)==0)
        continue
    end
    idx = labels==u(i);
    p = cloud(idx,1:3);
    myplot3(p,'.','color',rand(1,3));
end
hold off
axis equal
view3d on
return

figure
hold on
plot3k(cloud(:,1:3),'ColorData',labels,'Marker',{'.',5});
hold off
axis equal;
view3d on



%% hulls
block_size = [15+3/8.0, 15+3/8.0, 5+5/8.0]*0.0254;
block_size(3) = block_size(3)*2;
block_vertices = [
    -1,-1,-1
    1,-1,-1
    1,1,-1
    -1,1,-1
    -1,-1,1
    1,-1,1
    1,1,1
    -1,1,1
    ];
block_vertices(:,3) = block_vertices(:,3)-1;
block_vertices = block_vertices*0.5*diag(block_size);
block_faces = [
    1,2,3,4
    5,6,7,8
    1,2,6,5
    2,3,7,6
    3,4,8,7
    4,1,5,8
    ];

basedir = '/home/antone/drc/software/perception/plane-seg';
boxdata = load([basedir,'/boxes.txt']);
hulldata = load([basedir,'/hulls.txt']);
pose_data = load([basedir,'/poses.txt']);
cloud_full = loadpcd([basedir,'/cloud_full.pcd'])';
cloud = loadpcd([basedir,'/cloud.pcd'])';

poses = cell(size(pose_data,1)/4,1);
for i = 1:size(pose_data,1)/4
    poses{i} = pose_data(i*4-3:i*4,:);
end

%p = load('/home/antone/data/tilted-steps.csv');
%p = load('/home/antone/data/2015-03-11_testbed/terrain_med.csv');
%p = load('/home/antone/data/2015-03-11_testbed/terrain_close.csv');
p = loadpcd('/home/antone/temp/cloud.pcd')';
%p = loadpcd('/home/antone/temp/cloud_full.pcd')';
pts = p(:,1:3);
pts = cloud(:,1:3);
figure
plot3k(pts(:,1:3),'Marker',{'.',5});
hold on

u = unique(boxdata(:,1));
for i = 1:numel(u)
    p = boxdata(boxdata(:,1)==u(i),2:4);
    myplot3(p([1:end,1],:),'r.-','linewidth',4);
end

u = unique(hulldata(:,1));
for i = 1:numel(u)
    p = hulldata(hulldata(:,1)==u(i),2:4);
    myplot3(p([1:end,1],:),'g.-','linewidth',4);
end

for i = 1:numel(poses)
    pts = [block_vertices,ones(size(block_vertices,1),1)]*poses{i}(1:3,:)';
    x = reshape(pts(block_faces,1),size(block_faces));
    y = reshape(pts(block_faces,2),size(block_faces));
    z = reshape(pts(block_faces,3),size(block_faces));
    %plot3(x(:,[1:end,1])',y(:,[1:end,1])',z(:,[1:end,1])','r:','linewidth',3);
    %plot3(x(:,[1:end,1])',y(:,[1:end,1])',z(:,[1:end,1])','r-','linewidth',4);
end

hold off;
axis equal
view3d on
