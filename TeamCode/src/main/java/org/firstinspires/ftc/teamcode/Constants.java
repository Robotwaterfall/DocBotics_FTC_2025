package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    // =======Limelight Settings=======
    public static  double camera_Height        = 0.52;
    public static  double target_Height        = 0.82;
    public static  double camera_Angle         = 85.0;

    // =======PID Gains=======
    public static  double kPRotation           = 0.02;

    // =======Tolerance and Deadband=======
    public static  double tolerance            = 0.05; //offset of the rotation and forwarding
    public static  double deadband             = 1.5; //degrees in which we stop rotating

    // =======Target Setpoints=======
    public static  double targetDistanceMeters = 2;


    private Constants() { }

}
