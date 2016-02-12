function plot_floating_base(bag_name)

 msgs = read_introspection_bag(bag_name);
 Q = read_introspection_variable(msgs, 'Q', 'vectors');
 QDot = read_introspection_variable(msgs, 'QDot', 'vectors');
 NumDiffQDot = read_introspection_variable(msgs, 'floating_base_numerical_diff', 'vectors');

 %QDDot = read_introspection_variable(msgs, 'QDDot', 'vectors');

subplot(3,2,1);
hold on;
plot(Q(:, 1:3));
legend('Q linear X', 'Q linear Y', 'Q linear Z');
grid on;
hold off;

subplot(3,2,2);
hold on;
plot(Q(:, 4:6));
legend('Q angular X', 'Q angular Y', 'Q angular Z');
grid on;
hold off;

subplot(3,2,3);
hold on;
plot(QDot(:, 1:3));
legend('QDot linear X', 'QDot linear Y', 'QDot linear Z');
grid on;
hold off;


subplot(3,2,4);
hold on;
plot(QDot(:, 4:6));
plot(NumDiffQDot(:, 4:6));
legend('QDot angular X', 'QDot angular Y', 'QDot angular Z','DIFFQDot angular X', 'DIFFQDot angular Y', 'DIFFQDot angular Z' );
grid on;
hold off;

subplot(3,2,5);
hold on;
plot(QDot(:, 1:3) - NumDiffQDot(:, 1:3));
legend('DIFFQDot linear X', 'DIFFQDot linear Y', 'DIFFQDot linear Z' );
grid on;
hold off;


subplot(3,2,6);
hold on;
plot(QDot(:, 4:6) - NumDiffQDot(:, 4:6));
legend('DIFFQDot angular X', 'DIFFQDot angular Y', 'DIFFQDot angular Z' );
grid on;
hold off;



end
