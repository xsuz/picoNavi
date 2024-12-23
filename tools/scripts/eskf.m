
imuFs = 50;
gpsFs = 10;

imu_log=readtable("imu.csv");
gps_log=readtable("gps.csv");

fusionfilt = insfilterMARG;
fusionfilt.IMUSampleRate=imuFs;
fusionfilt.ReferenceLocation=[gps_log(1,'lat').lat,gps_log(1,'lng').lng,gps_log(1,'alt').alt];

% Initialize the states of the filter 

init_state=zeros(22,1);
init_state(1:4)=[1.0, 0.0, 0.0, 0.0]; % orientation as a quaternion
init_state(11:13)=deg2rad(0.0)/imuFs;
init_state(14:16)=0.00/imuFs;

fusionfilt.State=init_state;
% Measurement noises
Rmag = 0.0862; % Magnetometer measurement noise
Rvel = 0.011; % GPS Velocity measurement noise
Rpos = 3.0; % GPS Position measurement noise

% Process noises
fusionfilt.AccelerometerBiasNoise =  0.010716; 
fusionfilt.AccelerometerNoise = 230e-6; 
fusionfilt.GyroscopeBiasNoise = 1.3436e-14; 
fusionfilt.GyroscopeNoise =  0.015;
fusionfilt.MagnetometerBiasNoise = 2.189e-11;
fusionfilt.GeomagneticVectorNoise = 7.67e-13;

% Initial error covariance
fusionfilt.StateCovariance = 1e-12*eye(22);

idx_imu=100;
idx_gps=100;
len_imu=max(size(imu_log));
len_gps=max(size(gps_log));


% Log data for final metric computation.
pqorient = quaternion.zeros(len_imu,1);
pqpos = zeros(len_imu,3);

p1=poseplot(ones('quaternion'),[0 0 0]);
%p2=poseplot(ones('quaternion'),[0 0 0]);

while (idx_imu<=len_imu) && (idx_gps<=len_gps)
    if imu_log.timestamp(idx_imu)<gps_log.timestamp(idx_gps)
        accel=[imu_log.ax(idx_imu),imu_log.ay(idx_imu),imu_log.az(idx_imu)];
        gyro=[imu_log.wx(idx_imu),imu_log.wy(idx_imu),imu_log.wz(idx_imu)];%/pi*180.0;
        mag=[imu_log.mx(idx_imu),imu_log.my(idx_imu),imu_log.mz(idx_imu)];
        %if (imu_log(idx_imu,1)>5.078e7)
            % update by sys eq
            predict(fusionfilt,accel,gyro);
            [fusedPos,fusedOrient]=pose(fusionfilt);

        % Save the position and orientation for post processing.
            pqorient(idx_imu) = fusedOrient;
            pqpos(idx_imu,:) = fusedPos;
            %set(p1,Orientation=fusedOrient,Position=fusedPos);
            %set(p2,Orientation=quaternion(imu_log.q0(idx_imu),imu_log.q1(idx_imu),imu_log.q2(idx_imu),imu_log.q3(idx_imu)),Position=fusedPos);
            drawnow
        %end
        idx_imu=idx_imu+1;
    else
        %if (gps_log(idx_imu)>5.078e7)
        % update by obs eq
        lla=[gps_log.lat(idx_gps),gps_log.lng(idx_gps),gps_log.alt(idx_gps)];
        gpsvel=[gps_log.ve(idx_gps),gps_log.ve(idx_gps),0];
        fusegps(fusionfilt,lla,Rpos,gpsvel,Rvel);
        %fusemag(fusionfilt,mag,Rmag);
        %end
        idx_gps=idx_gps+1;
    end
end
hold off

figure

peuler=euler(quaternion(pqorient),'ZYX','frame');
peuler=rad2deg(peuler);
hold on
plot(imu_log.timestamp,peuler(:,1));
plot(imu_log.timestamp,peuler(:,2));
plot(imu_log.timestamp,peuler(:,3));
legend("yaw","pitch","roll")