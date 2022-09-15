I2C_BUS_ID = 1
BMI160_ADDR = 0x68

IMU_MSG_TOPIC = "data_raw"
ODOM_MSG_TOPIC = "odom_zero"
IMU_MSG_RATE = 25
ACCEL_RANGE_4G_FLOAT = 4.0
GYRO_RANGE_250_FLOAT = 250.0
CONVERSION_MASK_16BIT_FLOAT = 0x8000
GRAVITY_CONSTANT = 9.80665
EMPTY_ARRAY_9 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
COVAR_ARRAY_9 = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
EMPTY_ARRAY_36 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
