package org.firstinspires.ftc.teamcode;

public class Constants {

    // =======Limelight Settings=======
    public static final double camera_Height        = 0.52;
    public static final double target_Height        = 0.82;
    public static final double camera_Angle         = 75.0;

    // =======PID Gains=======
    public static final double kPForward            = 0.6;
    public static final double kPRotation           = 0.05;
    public static final double kPStrafe             = 0.05;

    // =======Tolerance and Deadband=======
    public static final double tolerance            = 0.05; //offset of the rotation and forwarding
    public static final double deadband             = 1.0; //degrees in which we stop strafing

    // =======Target Setpoints=======
    public static final double targetDistanceMeters = 2;


    private Constants() { }

}
