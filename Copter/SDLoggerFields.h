#ifndef __LOGGER_FIELDS
#define __LOGGER_FIELDS

/**
 * THIS FILE IS PARSED BY JAVA
 * To determine fields by their integer representation
 */

#define FIELD_ERROR 0 //p{-50;50}
#define FIELD_DIREVATIVE 1 //d{-50;50}
#define FIELD_X_ANGLE 2 //xAngle{90;270}
#define FIELD_Y_ANGLE 3 //yAngle{90;270}
#define FIELD_X_PID_SPEED 4 //xPIDSpeed{-40;40}
#define FIELD_Y_PID_SPEED 5 //yPIDSpeed{-40;40}
#define FIELD_GYRO_X 6 //gyro_x{-2000;2000}
#define FIELD_GYRO_Y 7 //gyro_y{-2000;2000}
#define FIELD_GYRO_Z 8 //gyro_z{-2000;2000}
#define FIELD_ACC_X 9 //acc_x{-2000;2000}
#define FIELD_ACC_Y 10 //acc_y{-2000;2000}
#define FIELD_ACC_Z 11 //acc_z{-2000;2000}
#define FIELD_FUZZY_D 12 //fuzzy_d{0;2}
#define FIELD_PID_TARGET_X 13 //pid_target_x{0;2}
#define FIELD_PID_TARGET_Y 14 //pid_target_y{0;2}
#define FIELD_X_ANGLE_PREDICT 15 //x_angle_predict{0;2}
#define FIELD_Y_ANGLE_PREDICT 16 //y_angle_predict{0;2}
#define FIELD_ANGLE_ROT_PREDICT 17 //angle_rotate_predict{0;2}


#endif