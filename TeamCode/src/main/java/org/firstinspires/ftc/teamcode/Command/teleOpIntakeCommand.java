package org.firstinspires.ftc.teamcode.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.Subsystem.intakeSubsystem;

public class teleOpIntakeCommand extends CommandBase {
    intakeSubsystem intakeSub;

    public teleOpIntakeCommand(intakeSubsystem intakeSub){
        this.intakeSub = intakeSub;
    }

    @Override
    public void initialize(){
        intakeSub.setM_intakeMotorPower(0);
    }

    @Override
    public void execute(){
        intakeSub.setM_intakeMotorPower(0.8); // sets motor power to 80%
    }
}
