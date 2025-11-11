package org.firstinspires.ftc.teamcode.Command;

import static org.firstinspires.ftc.teamcode.Constants.cataPower;
import static org.firstinspires.ftc.teamcode.Constants.cata_Gear_Reduction;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystem.catapultSubsystem;

public class catapultCommand extends CommandBase {

    private final catapultSubsystem cataSub;
    private final int cataSetpointDeg;
    private MotorGroup catapult;

    public catapultCommand(catapultSubsystem cataSub, int cataSetpointDeg){
        this.cataSub = cataSub;
        this.cataSetpointDeg = cataSub.getCataSetpoint();
        addRequirements(cataSub);

    }

    @Override
    public void initialize() {
        cataSub.getM_catapult1().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cataSub.getM_catapult2().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    @Override
    public void execute() {
        DcMotor cata1Motor = cataSub.getM_catapult1();
        DcMotor cata2Motor = cataSub.getM_catapult2();

        int currentPosition = cata1Motor.getCurrentPosition();

        double cataTarget;

        double Ticks_Per_Output_Rev = currentPosition * cata_Gear_Reduction;
        double degress_Per_Tick = 360 / Ticks_Per_Output_Rev;


        cataTarget =  cataSetpointDeg * (degress_Per_Tick);

        cata1Motor.setTargetPosition((int) cataTarget);
        cata1Motor.setTargetPosition((int) cataTarget);

        cataSub.runToPosition();

        cata1Motor.setPower(cataPower); //cata power
        cata2Motor.setPower(cataPower);


    }

    public boolean isFinished(){
        return !cataSub.getM_catapult1().isBusy() || !cataSub.getM_catapult2().isBusy();
    }
}
