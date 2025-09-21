package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Command.drive_straight_to_distanceCMD;
import org.firstinspires.ftc.teamcode.Command.strafe_to_targetCMD;
import org.firstinspires.ftc.teamcode.Command.teleOpMecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystem.limelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

@TeleOp(name = "TeleOpMode")
public class RobotContainer extends CommandOpMode {
    private mecanumDriveSubsystem driveSub;
    private limelightSubsystem llSub;
    private GamepadEx driverJoystick;

    @Override
    public void initialize() {

        // Mecanum Motor binding
        driveSub = new mecanumDriveSubsystem(
                new Motor(hardwareMap, "front_left"),
                new Motor(hardwareMap, "front_right"),
                new Motor(hardwareMap, "back_left"),
                new Motor(hardwareMap, "back_right"),
                hardwareMap
        );

        llSub = new limelightSubsystem(hardwareMap);

        driverJoystick = new GamepadEx(gamepad1);

        runCommands();
        setDefaultCommands();
    }

    /**
     * Apply a joystick deadband so tiny inputs don’t move the motors.
     *
     * @param value     joystick value
     * @param threshold minimum absolute value to count as input
     * @return filtered value
     */
    private double applyDeadband(double value, double threshold) {
        return (Math.abs(value) > threshold) ? value : 0.0;
    }

    public void setDefaultCommands() {
        /*
         * Sets the joysticks to always work to drive the robot
         * unless a different Op mode is selected
         */
        driveSub.setDefaultCommand(
                new teleOpMecanumDriveCommand(
                        driveSub,
                        () -> applyDeadband(driverJoystick.getLeftY(), 0.05),  // Forward/back
                        () -> applyDeadband(driverJoystick.getLeftX(), 0.05),  // Strafe
                        () -> applyDeadband(driverJoystick.getRightX(), 0.05) // Rotate
                )
        );

        driverJoystick.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> {
                    // Schedule the command manually
                    new strafe_to_targetCMD(driveSub, llSub).schedule();
                });
        driverJoystick.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> {

                    new drive_straight_to_distanceCMD(driveSub, llSub, Constants.targetDistanceMeters).schedule();
                });


    }

    private void runCommands() {
        // Add other commands here if needed
    }
}