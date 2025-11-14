package org.firstinspires.ftc.teamcode.Command;

import static org.firstinspires.ftc.teamcode.Constants.CATA_HOLDPOWER;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.Subsystem.catapultSubsystem;

public class holdCatapultCMD extends CommandBase {
    private final catapultSubsystem cataSub;
    DcMotor cataMotor, cata2Motor;

    public holdCatapultCMD(catapultSubsystem cataSub){
        this.cataSub = cataSub;
        this.cataMotor = cataSub.getM_catapult1();
        this.cata2Motor = cataSub.getM_catapult2();
        addRequirements(cataSub);
    }

    @Override
    public void initialize() {
        cataSub.stopAndRestEncoder();

        cataSub.getM_catapult1().setPower(0);
        cataSub.getM_catapult2().setPower(0);

        cataSub.runUsingEncoder();

        cataMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cata2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void execute() {
        int holdPosition = cataMotor.getCurrentPosition();
        cataMotor.setTargetPosition(holdPosition);
        cata2Motor.setTargetPosition(holdPosition);

        cataMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cata2Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        cataMotor.setPower(CATA_HOLDPOWER);
        cata2Motor.setPower(CATA_HOLDPOWER);
    }

    @Override
    public boolean isFinished() {
        return !cataSub.getM_catapult1().isBusy() && !cataSub.getM_catapult2().isBusy();
    }
}
