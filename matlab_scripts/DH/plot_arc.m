close all
plot3(j1.x, j1.y, j1.z, 'r', 'LineWidth',2)
hold on;
grid on;
plot3(j2.x, j2.y, j2.z, 'g', 'LineWidth',2)
plot3(j3.x, j3.y, j3.z, 'b', 'LineWidth',2)
plot3(j4.x, j4.y, j4.z, 'c', 'LineWidth',2)
plot3(j5.x, j5.y, j5.z, 'y', 'LineWidth',2)
plot3(j6.x, j6.y, j6.z, 'm', 'LineWidth',2)

title("ARCOS")
xlabel('X')
ylabel('Y')
zlabel('Z')
legend('J1', 'J2', 'J3', 'J4', 'J5', 'J6');
xlim([-0.8 0.6])
ylim([-0.8 0.6])
zlim([-0.0 1.0])
view(70,20)