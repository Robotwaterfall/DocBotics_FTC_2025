package org.firstinspires.ftc.teamcode.Command;

import static org.firstinspires.ftc.teamcode.Constants.catapult_up_power;
import static org.firstinspires.ftc.teamcode.Subsystem.catapultSubsystem.pivotMode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.catapultSubsystem;

import java.sql.Time;

public class catapultUpCommand extends CommandBase {

    private catapultSubsystem cataSub;

    private ElapsedTime cataUpTime = new ElapsedTime();

    public catapultUpCommand(catapultSubsystem cataSub){
        this.cataSub = cataSub;
        addRequirements(cataSub);
    }

    @Override
    public void initialize(){

        //when initialized catapult starts with motor brakes on
        cataSub.getM_catapult1().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cataSub.getM_catapult2().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //resets the catapults run time
        cataUpTime.reset();

    }

    @Override
    public void execute(){
        //starts the catapults clock going up
        cataUpTime.startTime();

        //sets the catapults position
        pivotMode = catapultSubsystem.CatapultModes.UP;

        //sets the motor powers
        cataSub.getM_catapult1().setPower(catapult_up_power);
        cataSub.getM_catapult2().setPower(catapult_up_power);
    }

    @Override
    public boolean isFinished(){
        if(cataUpTime.seconds() == 0.8 && pivotMode.equals(catapultSubsystem.CatapultModes.UP)){
            return true;
        } else {
            return false;
        }
    }
}
