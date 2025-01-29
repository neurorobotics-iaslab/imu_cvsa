# imu_cvsa

It uses the Mtw imu.

Can be used with just one IMU sensor and the node start if it is found.

The service: '/imu_cvsa/receiving_singals' says if the system reseived at least one signal from the imu.

The data are published in '/imu_cvsa'. The type of the message is imu_data.msg

It work with the SDK of awinda imu sensor of 2022