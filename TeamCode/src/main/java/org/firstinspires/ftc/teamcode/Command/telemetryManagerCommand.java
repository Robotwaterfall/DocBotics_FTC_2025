package org.firstinspires.ftc.teamcode.Command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystem.telemetryManagerSubsystem;

public class telemetryManagerCommand extends CommandBase {

    private final telemetryManagerSubsystem teleSub;

    public telemetryManagerCommand(telemetryManagerSubsystem teleSub){
        this.teleSub = teleSub;
        addRequirements(teleSub);
    }

    @Override
    public void execute() {
       teleSub.runTelemetry();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
