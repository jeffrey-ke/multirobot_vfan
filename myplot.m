robot1x = squeeze(out.robot1xy{1}.Values.Data(1,1,:));
robot1y = squeeze(out.robot1xy{2}.Values.Data(1,1,:));
% plot(robot1x, robot1y, 'r*');
% hold on
robot2x = squeeze(out.robot2xy{1}.Values.Data(1,1,:));
robot2y = squeeze(out.robot2xy{2}.Values.Data(1,1,:));
% plot(robot2x, robot2y, 'b*');

robot3x = squeeze(out.robot3xy{1}.Values.Data(1,1,:));
robot3y = squeeze(out.robot3xy{2}.Values.Data(1,1,:));
% plot(robot3x, robot3y, 'g*');
% hold off

% todo:
% Make robots holonomic. Ask Kitts if this is allowed, and if not, how to
% solve the issue of individual robot heading.

num_time_steps = size(robot1x, 1);

figure;
xlabel("X-position");
ylabel("Y-position");
title("Robot cluster evolution over time");
grid on;
hold on;

for t = 1:num_time_steps
        r1x = robot1x(t);
        r1y = robot1y(t);

        r2x = robot2x(t);
        r2y = robot2y(t);

        r3x = robot3x(t);
        r3y = robot3y(t);
        
        plot([r3x, r1x,r2x], [r3y, r1y, r2y], '-o')

        pause(0.20);
end

