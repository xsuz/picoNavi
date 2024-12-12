
imuFs = 50;
gpsFs = 10;

imu_log=readmatrix("imu.csv");
imu_log(5:7)=rad2deg(imu_log(5:7)); % rps -> dps
gps_log=readmatrix("gps.csv");

fusionfilt = insfilterMARG;
fusionfilt.IMUSampleRate=imuFs;
fusionfilt.ReferenceLocation=gps_log(1,2:4);

% Initialize the states of the filter 

init_state=zeros(22,1);
init_state(1:4)=[1.0, 0.0, 0.0, 0.0]; % orientation as a quaternion
init_state(11:13)=deg2rad(0.0)/imuFs;
init_state(14:16)=0.19/imuFs;

fusionfilt.State=init_state;
% Measurement noises
Rmag = 0.0862; % Magnetometer measurement noise
Rvel = 0.051; % GPS Velocity measurement noise
Rpos = 5.169; % GPS Position measurement noise

% Process noises
fusionfilt.AccelerometerBiasNoise =  0.010716; 
fusionfilt.AccelerometerNoise = 230e-6; 
fusionfilt.GyroscopeBiasNoise = 1.3436e-14; 
fusionfilt.GyroscopeNoise =  0.015;
fusionfilt.MagnetometerBiasNoise = 2.189e-11;
fusionfilt.GeomagneticVectorNoise = 7.67e-13;

% Initial error covariance
fusionfilt.StateCovariance = 1e-9*eye(22);

idx_imu=100;
idx_gps=100;
len_imu=length(imu_log);
len_gps=length(gps_log);


% Log data for final metric computation.
pqorient = quaternion.zeros(len_imu,1);
pqpos = zeros(len_imu,3);

while (idx_imu<=len_imu) & (idx_gps<=len_gps)
    if imu_log(idx_imu,1)<gps_log(idx_gps,1)
        accel=imu_log(idx_imu,2:4);
        gyro=imu_log(idx_imu,5:7);
        mag=imu_log(idx_imu,8:10);
        %if (imu_log(idx_imu,1)>5.078e7)
            % update by sys eq
            predict(fusionfilt,accel,gyro);
            [fusedPos,fusedOrient]=pose(fusionfilt);

        % Save the position and orientation for post processing.
            pqorient(idx_imu) = fusedOrient;
            pqpos(idx_imu,:) = fusedPos;
        %end
        idx_imu=idx_imu+1;
    else
        %if (gps_log(idx_imu)>5.078e7)
        % update by obs eq
        lla=gps_log(idx_gps,2:4);
        gpsvel=gps_log(idx_gps,5:7);
        fusegps(fusionfilt,lla,Rpos,gpsvel,Rvel);
        fusemag(fusionfilt,mag,Rmag);
        %end
        idx_gps=idx_gps+1;
    end
end


peuler=euler(quaternion(pqorient),'ZYX','frame');
peuler=rad2deg(peuler);
hold on
plot(imu_log(:,1),peuler(:,1));
plot(imu_log(:,1),peuler(:,2));
plot(imu_log(:,1),peuler(:,3));
legend("yaw","pitch","roll")