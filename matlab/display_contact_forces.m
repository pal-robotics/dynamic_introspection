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
plot(RawLeftRawForce(:,1), 'r');
plot(ComputedLeftForce(:,1), 'b');
legend('computed left Force X','raw Force X');
hold off;

subplot(4,3,2);
hold on;
plot(RawLeftRawForce(:,2), 'r');
plot(ComputedLeftForce(:,2), 'b');
legend('computed left Force Y','raw Force Y');
hold off;

subplot(4,3,3);
hold on;
plot(RawLeftRawForce(:,3), 'r');
plot(ComputedLeftForce(:,3), 'b');
legend('computed left Force Z','raw Force Z');
hold off;

%%%%%%%%%%%%%%%%

subplot(4,3,4);
hold on;
plot(RawLeftRawTorque(:,1), 'r');
plot(ComputedLeftTorque(:,1), 'b');
legend('computed left Torque X','raw Torque X');
hold off;

subplot(4,3,5);
hold on;
plot(RawLeftRawTorque(:,2), 'r');
plot(ComputedLeftTorque(:,2), 'b');
legend('computed left Torque Y','raw Torque Y');
hold off;

subplot(4,3,6);
hold on;
plot(RawLeftRawTorque(:,3), 'r');
plot(ComputedLeftTorque(:,3), 'b');
legend('computed left Torque Z','raw Torque Z');
hold off;

%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%

subplot(4,3,7);
hold on;
plot(RawRightRawForce(:,1), 'r');
plot(ComputedRightForce(:,1), 'b');
legend('computed right Force X','raw Force X');
hold off;

subplot(4,3,8);
hold on;
plot(RawRightRawForce(:,2), 'r');
plot(ComputedRightForce(:,2), 'b');
legend('computed right Force Y','raw Force Y');
hold off;

subplot(4,3,9);
hold on;
plot(RawRightRawForce(:,3), 'r');
plot(ComputedRightForce(:,3), 'b');
legend('computed right Force Z','raw Force Z');
hold off;

%%%%%%%%%%%%%%%%

subplot(4,3,10);
hold on;
plot(RawLeftRawTorque(:,1), 'r');
plot(ComputedRightTorque(:,1), 'b');
legend('computed right Torque X','raw Torque X');
hold off;

subplot(4,3,11);
hold on;
plot(RawLeftRawTorque(:,2), 'r');
plot(ComputedRightTorque(:,2), 'b');
legend('computed right Torque Y','raw Torque Y');
hold off;

subplot(4,3,12);
hold on;
plot(RawLeftRawTorque(:,3), 'r');
plot(ComputedRightTorque(:,3), 'b');
legend('computed right Torque Z','raw Torque Z');
hold off;
end