package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Command.rotateToTargetCMD;
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
                        driveSub, llSub,
                        () -> applyDeadband(driverJoystick.getLeftY(), 0.05),  // Forward/back
                        () -> applyDeadband(driverJoystick.getLeftX(), 0.05),  // Strafe
                        () -> applyDeadband(driverJoystick.getRightX(), 0.05) // Rotate
                )
        );

        /*
        commented out the rotate to target command for testing purposes
        **/

        //driverJoystick.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
        //        .whenPressed(() -> {
                    // Schedule the command manually
        //            new rotateToTargetCMD(driveSub, llSub).schedule();
        //        });


    }

    private void runCommands() {
        // Add other commands here if needed
    }
}