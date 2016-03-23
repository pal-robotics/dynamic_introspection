msgs = read_introspection_bag('/home/hilario/walking.bag');
zmpy = read_introspection_variable(msgs, 'zmp_traj_y', 'vectors');

figure,
hold on;
grid on;

for(i=30:1000)
    
    plot(zmpy(i, :), '--')
    pause(0.4);

    grid on;
end