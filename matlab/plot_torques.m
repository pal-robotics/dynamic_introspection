function plot_torques(bag_name)

msgs = read_introspection_bag(bag_name);
value = read_introspection_variable(msgs, 'Torques', 'vectors');

figure
hold on;
for i = 1:size(value, 2)
    plot(value(:, i), 'color', rand(1,3));
end
ylim([-1000 1000]);

hold off;