package org.firstinspires.ftc.teamcode.Command;

import static org.firstinspires.ftc.teamcode.Constants.intake_POWER;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystem.intakeSubsystem;

import java.util.function.BooleanSupplier;

public class teleOpIntakeCommand extends CommandBase {
    intakeSubsystem intakeSub;
    private boolean toggleState = false;
    private boolean lastToggleState = false;
    BooleanSupplier buttonSupplier;

    public teleOpIntakeCommand(intakeSubsystem intakeSub, BooleanSupplier buttonSupplier){
        this.intakeSub = intakeSub;
        this.buttonSupplier = buttonSupplier;
        addRequirements(intakeSub);
    }

    @Override
    public void initialize(){
            intakeSub.setM_intakeMotorPower(0);

    }

    @Override
    public void execute(){
        boolean currentButtonState = buttonSupplier.getAsBoolean();
        if(currentButtonState && !lastToggleState){
            toggleState = !toggleState;
        }
        lastToggleState = currentButtonState;

        if(toggleState){
            intakeSub.setM_intakeMotorPower(intake_POWER); //power set to 80%
        } else {
            intakeSub.setM_intakeMotorPower(0);
        }

    }

    @Override
    public void end(boolean interrupted) {
        intakeSub.setM_intakeMotorPower(0);
    }
}
