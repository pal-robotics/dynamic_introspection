function display_contact_forces(bag_name)

msgs = read_introspection_bag(bag_name);

RawLeftRawForce = read_introspection_variable(msgs, 'left_raw_force', 'vectors3d');
RawRightRawForce = read_introspection_variable(msgs, 'right_raw_force', 'vectors3d');
RawLeftRawTorque = read_introspection_variable(msgs, 'left_raw_torque', 'vectors3d');
RawRightRawTorqe = read_introspection_variable(msgs, 'right_raw_torque', 'vectors3d');

ComputedLeftForce = read_introspection_variable(msgs, 'left_computed_force', 'vectors3d');
ComputedRightForce = read_introspection_variable(msgs, 'right_computed_force', 'vectors3d');
ComputedLeftTorque = read_introspection_variable(msgs, 'left_computed_torque', 'vectors3d');
ComputedRightTorque = read_introspection_variable(msgs, 'right_computed_torque', 'vectors3d');

figure
subplot(4,3,1);
hold on;
plot(ComputedLeftForce(:,1), 'b');
plot(RawLeftRawForce(:,1), 'r');
legend('computed left Force X','raw Force X');
grid on;
hold off;

subplot(4,3,2);
hold on;
plot(ComputedLeftForce(:,2), 'b');
plot(RawLeftRawForce(:,2), 'r');
legend('computed left Force Y','raw Force Y');
grid on;
hold off;

subplot(4,3,3);
hold on;
plot(ComputedLeftForce(:,3), 'b');
plot(RawLeftRawForce(:,3), 'r');
legend('computed left Force Z','raw Force Z');
grid on;
hold off;

%%%%%%%%%%%%%%%%

subplot(4,3,4);
hold on;
plot(ComputedLeftTorque(:,1), 'b');
plot(RawLeftRawTorque(:,1), 'r');
legend('computed left Torque X','raw Torque X');
grid on;
hold off;

subplot(4,3,5);
hold on;
plot(ComputedLeftTorque(:,2), 'b');
plot(RawLeftRawTorque(:,2), 'r');
legend('computed left Torque Y','raw Torque Y');
grid on;
hold off;

subplot(4,3,6);
hold on;
plot(ComputedLeftTorque(:,3), 'b');
plot(RawLeftRawTorque(:,3), 'r');
legend('computed left Torque Z','raw Torque Z');
grid on;
hold off;

%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%

subplot(4,3,7);
hold on;
plot(ComputedRightForce(:,1), 'b');
plot(RawRightRawForce(:,1), 'r');
legend('computed right Force X','raw Force X');
grid on;
hold off;

subplot(4,3,8);
hold on;
plot(ComputedRightForce(:,2), 'b');
plot(RawRightRawForce(:,2), 'r');
legend('computed right Force Y','raw Force Y');
grid on;
hold off;

subplot(4,3,9);
hold on;
plot(ComputedRightForce(:,3), 'b');
plot(RawRightRawForce(:,3), 'r');
legend('computed right Force Z','raw Force Z');
grid on;
hold off;

%%%%%%%%%%%%%%%%

subplot(4,3,10);
hold on;
plot(ComputedRightTorque(:,1), 'b');
plot(RawRightRawTorqe(:,1), 'r');
legend('computed right Torque X','raw Torque X');
grid on;
hold off;

subplot(4,3,11);
hold on;
plot(ComputedRightTorque(:,2), 'b');
plot(RawRightRawTorqe(:,2), 'r');
legend('computed right Torque Y','raw Torque Y');
grid on;
hold off;

subplot(4,3,12);
hold on;
plot(ComputedRightTorque(:,3), 'b');
plot(RawRightRawTorqe(:,3), 'r');
legend('computed right Torque Z','raw Torque Z');
grid on;
hold off;
end