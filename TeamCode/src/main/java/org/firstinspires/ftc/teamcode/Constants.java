package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    // =======PID Gains=======
    public static double kPRotation            = 0.02;

    // =======Tolerance and Deadband=======
    public static double tolerance             = 0.05; //offset of the rotation and forwarding
    public static double deadband              = 1.5; //degrees in which we stop rotating

    // =======Target Setpoints=======
    public static double targetDistanceMeters     = 2;

    public static double catapult_up_power        = -1.0;
    public static double catapult_down_power      = 1.0;

    // =======Motor Configs==========



    private Constants() { }

}
