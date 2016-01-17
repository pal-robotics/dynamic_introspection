function plot_cop(bag_name)

msgs = read_introspection_bag(bag_name);

localCOPleftX = read_introspection_variable(msgs, 'local_left_cop_X', 'double');
localCOPleftY= read_introspection_variable(msgs, 'local_left_cop_Y', 'double');
rawLocalCOPleftX = read_introspection_variable(msgs, 'raw_local_left_cop_X', 'double');
rawLocalCOPleftY= read_introspection_variable(msgs, 'raw_local_left_cop_Y', 'double');

localCOPrightX = read_introspection_variable(msgs, 'local_right_cop_X', 'double');
localCOPrightY= read_introspection_variable(msgs, 'local_right_cop_Y', 'double');
rawLocalCOPrightX = read_introspection_variable(msgs, 'raw_local_right_cop_X', 'double');
rawLocalCOPrightY= read_introspection_variable(msgs, 'raw_local_right_cop_Y', 'double');

globalCOPX = read_introspection_variable(msgs, 'global_combined_cop_X', 'double');
globalCOPY = read_introspection_variable(msgs, 'global_combined_cop_Y', 'double');
rawGlobalCOPX = read_introspection_variable(msgs, 'raw_global_combined_cop_X', 'double');
rawGlobalCOPY= read_introspection_variable(msgs, 'raw_global_combined_cop_Y', 'double');

figure
subplot(2,2,1);
hold on;
plot(localCOPleftX, 'r');
plot(localCOPleftY, 'g');
plot(rawLocalCOPleftX, 'm');
plot(rawLocalCOPleftY);
legend('local COP left X','local COP left Y', 'RAW local COP left X','RAW local COP left Y');
hold off;

subplot(2,2,2);
hold on;
plot(localCOPrightX, 'r' );
plot(localCOPrightY, 'g');
plot(rawLocalCOPrightX, 'm' );
plot(rawLocalCOPrightY);
legend('local COP right X','local COP right Y', 'RAW local COP right X','RAW local COP right Y')
hold off;

subplot(2,2,3);
hold on;
plot(globalCOPX, 'r' );
plot(globalCOPY, 'g');
plot(rawGlobalCOPX, 'm' );
plot(rawGlobalCOPY);
legend('global COP X','global COP Y', 'RAW global COP X', 'RAW global COP Y');
hold off;

end