sc = trackingScenario;
rng(2020) % for repeatable results

plat1 = platform(sc);
plat2 = platform(sc);

target = platform(sc);

traj = waypointTrajectory("Waypoints",[1 1 1; 2 2 2],"TimeOfArrival",[0,1]);
%target.Trajectory = traj;

target.Mesh = extendedObjectMesh("Sphere");
target.Dimensions = struct("Length",4,"Width",3,"Height",2,"OriginOffset",[0 0 0]);

figure()
show(target.Mesh);
legend("Target Mesh")
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');

sensor1 = monostaticLidarSensor(1,"RangeAccuracy",0.01);
sensor2 = monostaticLidarSensor(2,"RangeAccuracy",0.2);
plat1.Sensors = {sensor1};
plat2.Sensors = {sensor2};

[pointClouds,configs,clusters] = lidarDetect(sc);

cloud1 = pointClouds{1};
cloud2 = pointClouds{2};
figure()
plot3(cloud1(:,1),cloud1(:,2),cloud1(:,3),'bo')
hold on
plot3(cloud2(:,1),cloud2(:,2),cloud2(:,3),'go')
legend('Sensor1','Sensor2')
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)')