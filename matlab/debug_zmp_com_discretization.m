msgs = read_introspection_bag('/home/hilario/walking.bag');
zmpy = read_introspection_variable(msgs, 'zmp_traj_y', 'vectors');
computed_zmpy = read_introspection_variable(msgs, 'compute_zmp_y', 'vectors');
comy = read_introspection_variable(msgs, 'com_traj_y', 'vectors');
time = read_introspection_variable(msgs, 'time', 'vectors');

com_traj_y_sub = read_introspection_variable(msgs, 'com_traj_y_sub', 'vectors');
zmp_traj_y_sub = read_introspection_variable(msgs, 'zmp_traj_y_sub', 'vectors');

time_sub = read_introspection_variable(msgs, 'time_sub', 'vectors');


figure,
hold on;
%grid on;

for(i=30:1000)
    clf;
    hold on;  
    grid on;

    plot(time(i, :), zmpy(i, :), '*--')
    plot(time(i, :), comy(i, :), 'r*--')
    plot(time(i, :), computed_zmpy(i, :), 'g*--')

    %plot(time_sub(i, :), com_traj_y_sub(i, :), 'c--')
    %plot(time_sub(i, :), zmp_traj_y_sub(i, :), 'r*--')

    
    pause(0.5);

    hold off;
end