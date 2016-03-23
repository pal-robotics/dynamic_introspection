function EstimatedPositionCovariance = reemc_estimation(bag_name)

msgs = read_introspection_bag(bag_name);

% Base variables
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

% Left leg variables
GroundTruthLeftFootPosition = read_introspection_variable(msgs, 'ground_truth_left_leg_position', 'vectors3d');
GroundTruthLeftFootOrientationRPY = read_introspection_variable(msgs, 'ground_truth_left_leg_orientation_rpy', 'vectors3d');
EstimatedLeftFootPosition = read_introspection_variable(msgs, 'left_foot_estimated_position', 'vectors3d');
EstimatedLeftFootCovariance = read_introspection_variable(msgs, 'left_foot_estimated_position_covariance', 'matrixs3d'); 
EstimatedLeftFootOrientationRPY = read_introspection_variable(msgs, 'left_foot_estimated_orientation_rpy', 'vectors3d');
EstimatedLeftFootOrientationRPYCovariance = read_introspection_variable(msgs, 'left_foot_estimated_orientation_rpy_covariance', 'matrixs3d');

% Right leg variables
GroundTruthRightFootPosition = read_introspection_variable(msgs, 'ground_truth_right_leg_position', 'vectors3d');
GroundTruthRightFootOrientationRPY = read_introspection_variable(msgs, 'ground_truth_right_leg_orientation_rpy', 'vectors3d');
EstimatedRightFootPosition = read_introspection_variable(msgs, 'right_foot_estimated_position', 'vectors3d');
EstimatedRightFootCovariance = read_introspection_variable(msgs, 'right_foot_estimated_position_covariance', 'matrixs3d'); 
EstimatedRightFootOrientationRPY = read_introspection_variable(msgs, 'right_foot_estimated_orientation_rpy', 'vectors3d');
EstimatedRightFootOrientationRPYCovariance = read_introspection_variable(msgs, 'right_foot_estimated_orientation_rpy_covariance', 'matrixs3d');

%%%% Display results

% Base link
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

% Left foot 
figure;

subplot(2,3,1);
hold on;
plot(GroundTruthLeftFootPosition(:,1), 'b');
plot(EstimatedLeftFootPosition(:,1), 'r');
legend('ground truth left foot position X','estimated left foot position X');
grid on;

subplot(2,3,2);
hold on;
plot(GroundTruthLeftFootPosition(:,2), 'b');
plot(EstimatedLeftFootPosition(:,2), 'r');
legend('ground truth left foot position Y','estimated left foot position Y');
grid on;

subplot(2,3,3);
hold on;
plot(GroundTruthLeftFootPosition(:,3), 'b');
plot(EstimatedLeftFootPosition(:,3), 'r');
legend('ground truth left foot position Z','estimated left foot position Z');
grid on;

subplot(2,3,4);
hold on;
plot(GroundTruthLeftFootOrientationRPY(:,1), 'b');
plot(EstimatedLeftFootOrientationRPY(:,1), 'r');
legend('ground truth left foot ROLL','estimated left foot ROLL');
grid on;

subplot(2,3,5);
hold on;
plot(GroundTruthLeftFootOrientationRPY(:,2), 'b');
plot(EstimatedLeftFootOrientationRPY(:,2), 'r');
legend('ground truth left foot PITCH','estimated left foot PITCH');
grid on;

subplot(2,3,6);
hold on;
plot(GroundTruthLeftFootOrientationRPY(:,3), 'b');
plot(EstimatedLeftFootOrientationRPY(:,3), 'r');
legend('ground truth left foot YAW','estimated left foot YAW');
grid on;

hold off;

% Right foot 
figure;

subplot(2,3,1);
hold on;
plot(GroundTruthRightFootPosition(:,1), 'b');
plot(EstimatedRightFootPosition(:,1), 'r');
legend('ground truth right foot position X','estimated right foot position X');
grid on;

subplot(2,3,2);
hold on;
plot(GroundTruthRightFootPosition(:,2), 'b');
plot(EstimatedRightFootPosition(:,2), 'r');
legend('ground truth right foot position Y','estimated right foot position Y');
grid on;

subplot(2,3,3);
hold on;
plot(GroundTruthRightFootPosition(:,3), 'b');
plot(EstimatedRightFootPosition(:,3), 'r');
legend('ground truth right foot position Z','estimated right foot position Z');
grid on;

subplot(2,3,4);
hold on;
plot(GroundTruthRightFootOrientationRPY(:,1), 'b');
plot(EstimatedRightFootOrientationRPY(:,1), 'r');
legend('ground truth right foot ROLL','estimated right foot ROLL');
grid on;

subplot(2,3,5);
hold on;
plot(GroundTruthRightFootOrientationRPY(:,2), 'b');
plot(EstimatedRightFootOrientationRPY(:,2), 'r');
legend('ground truth right foot PITCH','estimated right foot PITCH');
grid on;

subplot(2,3,6);
hold on;
plot(GroundTruthRightFootOrientationRPY(:,3), 'b');
plot(EstimatedRightFootOrientationRPY(:,3), 'r');
legend('ground truth right foot YAW','estimated right foot YAW');
grid on;

hold off;

end