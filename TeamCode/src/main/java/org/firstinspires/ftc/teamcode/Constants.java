package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    // motor powers
    public static double intake_POWER = 1.0;

//    powers in which the shooter has set too.
    public static double min_Shooter_Power = 0.5;
    public static  double max_Shooter_Power = 1.0;


    //Limelight
    public static double min_Ty = -10; //Constrains in which the shooter
                                        // doesnt react with the april tag
    public static double max_Ty = 10;



    public static final class encoderAutoConstants{
        public static final double     DRIVE_SPEED             = 0.25;
        public static final double     TURN_SPEED              = 0.5;

        public static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // REV ultraplanetary motor
        public static final double     DRIVE_GEAR_REDUCTION    = 20 ;     // External Gearing.
        public static final double     WHEEL_DIAMETER_INCHES   = 3 ;     // For figuring circumference
        public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * Math.PI);

    }





    private Constants() { }

}
