package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.cata_Down_setpoint;
import static org.firstinspires.ftc.teamcode.Constants.cata_Up_setpoint;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Command.catapultCommand;
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
                hardwareMap.get(DcMotor.class,"intake_Motor")
        );

        cataSub = new catapultSubsystem(
                hardwareMap.get(DcMotor.class, "CatapultMotor1"),
                hardwareMap.get(DcMotor.class, "CatapultMotor2")
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

        intakeSub.setDefaultCommand(
                new teleOpIntakeCommand(
                        intakeSub,
                        () -> driverJoystick.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                        () -> driverJoystick.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
                )
        );



        driverJoystick.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new SequentialCommandGroup(
                        new catapultCommand(cataSub, cata_Up_setpoint),
                        new catapultCommand(cataSub, cata_Down_setpoint)
                ));





    }

    private void runCommands() {
        // Add other commands here if needed


    }
}