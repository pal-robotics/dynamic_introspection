function plot_cop(bag_name)

msgs = read_introspection_bag(bag_name);

localCOPleftX = read_introspection_variable(msgs, 'local_left_cop_X', 'double');
localCOPleftY= read_introspection_variable(msgs, 'local_left_cop_Y', 'double');

localCOPrightX = read_introspection_variable(msgs, 'local_right_cop_X', 'double');
localCOPrightY= read_introspection_variable(msgs, 'local_right_cop_Y', 'double');

globalCOPX = read_introspection_variable(msgs, 'global_combined_cop_X', 'double');
globalCOPY= read_introspection_variable(msgs, 'global_combined_cop_Y', 'double');

figure
subplot(2,2,1);
hold on;
plot(localCOPleftX, 'r');
plot(localCOPleftY);
hold off;

subplot(2,2,2);
hold on;
plot(localCOPrightX, 'r' );
plot(localCOPrightY);
hold off;

subplot(2,2,3);
hold on;
plot(globalCOPX, 'r' );
plot(globalCOPY);
hold off;

end