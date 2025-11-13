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

    public static int cata_Up_setpoint = 6;
    public static int cata_Down_setpoint = -6;

    public static double RUBBER_BAND_FEEDFORWARD = 0.2; // Start low, tune higher if needed





    private Constants() { }

}
