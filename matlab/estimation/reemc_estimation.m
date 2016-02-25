function EstimatedPositionCovariance = reemc_estimation(bag_name)

msgs = read_introspection_bag(bag_name);

GroundTruthPosition = read_introspection_variable(msgs, 'ground_truth_position', 'vectors3d');
GroundTruthOrientationRPY = read_introspection_variable(msgs, 'ground_truth_linear_veloicity', 'vectors3d');
GroundTruthLinearVelocity = read_introspection_variable(msgs, 'ground_truth_linear_veloicity', 'vectors3d');
GroundTruthAngularVelocity =  read_introspection_variable(msgs, 'ground_truth_angular_velocity_rpy', 'vectors3d');

EstimatedPosition = read_introspection_variable(msgs, 'estimated_base_position', 'vectors3d');
EstimatedPositionCovariance = read_introspection_variable(msgs, 'estimated_base_position_covariance', 'matrixs3d');
EstimatedLinearVelocity = read_introspection_variable(msgs, 'estimated_base_linear_velocity', 'vectors3d');
EstimatedLinearVelocityCovariance = read_introspection_variable(msgs, 'estimated_base_linear_velocity_covariance', 'matrixs3d'); 
EstimatedOrienation = read_introspection_variable(msgs, 'estimated_base_orientation_rpy', 'vectors3d');
EstimatedOrienationCovariance = read_introspection_variable(msgs, 'estimated_base_linear_velocity_covariance', 'matrixs3d');

% GroundTruthLeftFootPosition
% GroundTruthLeftFootOrientationRPY
% GroundTruthRightFootPosition
% GroundTruthRightFootOrientationRPY
% 
% EstimatedLeftFootPosition = ;
% EstimatedLeftFootCovariance = ; 
% EstimatedLeftFootOrientationRPY = ;
% EstimatedLeftFootOrientationRPYCovariance = ;
% EstimatedRightFootPosition = ;
% EstimatedRightFootPositionCovariance = 
% EstimatedRightFootOrientationRPY = ;
% EstimatedRightFootOrientationRPYCovariance = ;

%%%% Display results


figure
subplot(2,3,1);
hold on;
plot(GroundTruthPosition(:,1), 'b');
plot(EstimatedPosition(:,1), 'r');
legend('ground truth position X','estimated position X');
grid on;

subplot(2,3,2);
hold on;
plot(GroundTruthPosition(:,2), 'b');
plot(EstimatedPosition(:,2), 'r');
legend('ground truth position Y','estimated position Y');
grid on;

subplot(2,3,3);
hold on;
plot(GroundTruthPosition(:,3), 'b');
plot(EstimatedPosition(:,3), 'r');
legend('ground truth position Z','estimated position Z');
grid on;

subplot(2,3,4);
hold on;
plot(GroundTruthLinearVelocity(:,1), 'b');
plot(EstimatedLinearVelocity(:,1), 'r');
legend('ground truth velocity X','estimated velocity X');
grid on;

subplot(2,3,5);
hold on;
plot(GroundTruthLinearVelocity(:,2), 'b');
plot(EstimatedLinearVelocity(:,2), 'r');
legend('ground truth velocity Y','estimated velocity Y');
grid on;

subplot(2,3,6);
hold on;
plot(GroundTruthLinearVelocity(:,3), 'b');
plot(EstimatedLinearVelocity(:,3), 'r');
legend('ground truth velocity Z','estimated velocity Z');
grid on;

hold off;

end