package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    // motor powers
    public static double CATA_POWER = 1.0;


    public static double intake_POWER = 0.8;

    // Catapult
    public static final int MOTOR_TICKS_PER_REV = 560; // Change if using other motors
    public static final double CATA_GEAR_REDUCTION = 20.0; // As set in your config
    public static final int TICKS_PER_OUTPUT_REV = (int)(MOTOR_TICKS_PER_REV * CATA_GEAR_REDUCTION);
    public static final double DEGREES_PER_TICK = 360.0 / TICKS_PER_OUTPUT_REV;

    public static int cata_Up_setpoint = 0;
    //TODO Check Down position.
    public static int cata_Down_setpoint = -4;

    public static double RUBBER_BAND_FEEDFORWARD = 0.2; // Start low, tune higher if needed

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
