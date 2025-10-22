package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    public static double  soos_Linear_scaler = 1.1975;
    public static double  soos_Angular_scaler = 0.98622;

    //drivetrain feedforward constants

    //measured at 13.4V
    public static double drivetrain_ks_x = 0.09;
    //measured at 13.2V
    public static double drivetrain_kv_x = 0.00945;
    //measured at 13.2
    public static double drivetrain_ka_x = 0.00124;

    //measured at 13.4V
    public static double drivetrain_ks_y = 0.09;
    //measured at 13.2V
    public static double drivetrain_kv_y = 0.00945;
    //measured at 13.2
    public static double drivetrain_ka_y = 0.00124;

    //measured at 13.4V
    public static double drivetrain_ks_h = 0.09;
    //measured at 13.2V
    public static double drivetrain_kv_h = 0.00945;
    //measured at 13.2
    public static double drivetrain_ka_h = 0.00124;


    // Heading PID
    public static double heading_Kp = 0;
    public static double heading_Ki = 0;
    public static double heading_Kd = 0;



    //drivetrain Trapezoidal drive constants
    public static double drivetrain_maxAccel = 20.0; //inches/sec^2
    public static double drivetrain_maxVelocity = 30.0; //inches/sec
    public static double drivetrain_targetDistance = 45.0; // Measured in inches


    public static double linearMaxSpeedError = 0.001;
    public static double xSpeedController_kp = 0;
    public static double xSpeedController_kd = 0;

    public static double ySpeedController_kp = 0;
    public static double ySpeedController_kd = 0;

    public static double angularMaxSpeedError = 0.001;
    public static double angularSpeedController_kp = 0;
    public static double angularSpeedController_kd = 0;

    private Constants() { }

}
