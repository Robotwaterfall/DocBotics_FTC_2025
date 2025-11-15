package org.firstinspires.ftc.teamcode.Command;

import static org.firstinspires.ftc.teamcode.Constants.CATA_POWER;
import static org.firstinspires.ftc.teamcode.Constants.DEGREES_PER_TICK;
import static org.firstinspires.ftc.teamcode.Constants.RUBBER_BAND_FEEDFORWARD;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystem.catapultSubsystem;

public class catapultCommand extends CommandBase {

    private final catapultSubsystem cataSub;
    private final double cataSetpointDeg;

    public catapultCommand(catapultSubsystem cataSub, int cataSetpointDeg) {
        this.cataSub = cataSub;
        this.cataSetpointDeg = cataSetpointDeg;
        addRequirements(cataSub);

    }

    @Override
    public void initialize() {


        cataSub.getM_catapult1().setPower(0);
        cataSub.getM_catapult2().setPower(0);

        cataSub.runUsingEncoder();



    }

    @Override
    public void execute() {
        DcMotor cataMotor = cataSub.getM_catapult1();
        DcMotor cata2Motor = cataSub.getM_catapult2();


        // Convert degrees to encoder ticks
        double cataTargetTicks = cataSetpointDeg / DEGREES_PER_TICK;

        cataSub.setCataSetpoint((int) cataTargetTicks); // save or update setpoint in subsystem

        cataMotor.setTargetPosition((int) cataTargetTicks);
        cata2Motor.setTargetPosition((int) cataTargetTicks);

        cataSub.runToPosition(); // sets RUN_TO_POSITION mode for both motors

        cataMotor.setPower(CATA_POWER + RUBBER_BAND_FEEDFORWARD);
        cata2Motor.setPower(CATA_POWER + RUBBER_BAND_FEEDFORWARD);
    }

    public boolean isFinished() {
        // Both motors have reached their target positions
        return !cataSub.getM_catapult1().isBusy() && !cataSub.getM_catapult2().isBusy();
    }
}
