%% robot Trajectory
figure; hold on;
title('robot Trajectories');
plot(real_robot1(1,:),real_robot1(2,:),'r');
plot(real_robot2(1,:),real_robot2(2,:),'b');
plot(pos_robot1(1,:),pos_robot1(2,:),'r:');
plot(pos_robot2(1,:),pos_robot2(2,:),'b:');
grid on; xlabel('x (m)'); ylabel('y (m)');
axis([-30, 30, -30, 30]);
legend('True robot1','True robot2','Estimated robot1','Estimated robot2');

%% robot Position Error
error_robot1 = pos_robot1-real_robot1;
error_robot2 = pos_robot2-real_robot2;
rmse_robot1 = sqrt(sum(error_robot1.^2,1));
rmse_robot2 = sqrt(sum(error_robot2.^2,1));

figure; 
set(gcf,'position',[0 0 500 1000]);
subplot(311); hold on;
title('robot Estimation Error');
plot(time,rmse_robot1,'r');
plot(time,rmse_robot2,'b');
grid on; xlabel('Time (s)'); ylabel('RMSE [m]'); legend('robot1','robot2');

disp(['Average robot1 Position Estimation Error: ',num2str(mean(rmse_robot1))]);
disp(['Average robot2 Position Estimation Error: ',num2str(mean(rmse_robot2))]);

%% Landmark Position Error
error_landmark1 = pos_landmark1-real_landmarks;
error_landmark2 = pos_landmark2-real_landmarks;
error_landmark1 = reshape(error_landmark1(:),[2*num_landmarks,timesteps]);
error_landmark2 = reshape(error_landmark2(:),[2*num_landmarks,timesteps]);
rmse_landmark1 = sqrt(sum(error_landmark1.^2,1));
rmse_landmark2 = sqrt(sum(error_landmark2.^2,1));

subplot(312); hold on; 
title('Landmark Estimation Error');
plot(time,rmse_landmark1,'r');
plot(time,rmse_landmark2,'b');
grid on; xlabel('Time (s)'); ylabel('RMSE [m]'); legend('robot1','robot2');


disp(['Average Landmark Estimation Error from robot1: ',num2str(mean(rmse_landmark1))]);
disp(['Average Landmark Estimation Error from robot2: ',num2str(mean(rmse_landmark2))]);

if fusion_flag>0
    error_landmark_SF = pos_landmark_SF-real_landmarks;
    error_landmark_SF = reshape(error_landmark_SF(:),[2*num_landmarks,timesteps]);
    rmse_landmark_SF = sqrt(sum(error_landmark_SF.^2,1));
    plot(time,rmse_landmark_SF,'g');
    legend('robot1','robot2','Fusion');
    disp(['Average Landmark Estimation Error from Fusion: ',num2str(mean(rmse_landmark_SF))]);
end

%% Covariance Matrix
cov_landmark1 = reshape(cov_landmark1(:),[4*num_landmarks,timesteps]);
cov_landmark2 = reshape(cov_landmark2(:),[4*num_landmarks,timesteps]);
sum_cov_landmark1 = sqrt(sum(cov_landmark1.^2,1));
sum_cov_landmark2 = sqrt(sum(cov_landmark2.^2,1));

subplot(313); hold on; 
title('Landmark Covariance');
plot(time,sum_cov_landmark1,'r');
plot(time,sum_cov_landmark2,'b');
grid on; xlabel('Time (s)'); ylabel('Covariance [m^2]'); legend('robot1','robot2');

if fusion_flag>0
    cov_landmark_SF = reshape(cov_landmark_SF(:),[4*num_landmarks,timesteps]);
    sum_cov_landmark_SF = sqrt(sum(cov_landmark_SF.^2,1));
    plot(time,sum_cov_landmark_SF,'g');
    legend('robot1','robot2','Fusion');
end


