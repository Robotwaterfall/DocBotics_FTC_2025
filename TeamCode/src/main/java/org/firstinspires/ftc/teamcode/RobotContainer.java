package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Command.catapultDownCommand;
import org.firstinspires.ftc.teamcode.Command.catapultUpCommand;
import org.firstinspires.ftc.teamcode.Command.teleOpIntakeCommand;
import org.firstinspires.ftc.teamcode.Command.teleOpMecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystem.catapultSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

@TeleOp(name = "TeleOpMode")
public class RobotContainer extends CommandOpMode {
    private mecanumDriveSubsystem driveSub;
    private intakeSubsystem intakeSub;
    private catapultSubsystem cataSub;
    private TelemetryManager teleManager;
    private GamepadEx driverJoystick;

    @Override
    public void initialize() {

        // Mecanum Motor binding
        driveSub = new mecanumDriveSubsystem(
                hardwareMap.get(DcMotor.class,"front_left"),
                hardwareMap.get(DcMotor.class, "front_right"),
                hardwareMap.get(DcMotor.class, "back_left"),
                hardwareMap.get(DcMotor.class, "back_right"),
                hardwareMap
        );

        intakeSub = new intakeSubsystem(
                hardwareMap.get(DcMotor.class,"intake_Motor") //TODO: change intake motor name
        );

        cataSub = new catapultSubsystem(
                hardwareMap.get(DcMotor.class, "Catapult1Motor"), //TODO: change the catapult motor names
                hardwareMap.get(DcMotor.class, "Catapult2Motor")
        );


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

        new GamepadButton(driverJoystick, GamepadKeys.Button.A)
                .toggleWhenPressed(new teleOpIntakeCommand(intakeSub));

        // when the up command is not running cataput will be down
        cataSub.setDefaultCommand(
                new catapultDownCommand(cataSub)
        );

        new GamepadButton(driverJoystick, GamepadKeys.Button.B)
                .whenPressed(new catapultUpCommand(cataSub));

        teleManager.runTelemetry();

    }

    private void runCommands() {
        // Add other commands here if needed
    }
}