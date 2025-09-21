package org.firstinspires.ftc.teamcode.Command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.limelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

public class strafe_to_targetCMD extends CommandBase {
    private final mecanumDriveSubsystem drive;
    private final limelightSubsystem llSub;
    private final double kPStrafe = Constants.kPStrafe; //simple proportional gain
    private final double deadband = Constants.deadband; //degrees in which we stop
    public strafe_to_targetCMD(mecanumDriveSubsystem drive, limelightSubsystem llSub){
        this.drive = drive;
        this.llSub = llSub;
        addRequirements(drive);
    }

    @Override
    public void execute(){
        if(llSub.hasTarget()) {
            double error = llSub.getTx(); //horizontal offset in degrees
            double strafePower = -error * kPStrafe; //negative to correct direction
            //clipped power to [-0.4, 0.4] for safety
            strafePower = Math.max(Math.min(strafePower, 0.4), -0.4);

            drive.drive(0, strafePower, 0); //strafe only
        } else {
            drive.drive(0,0,0); //stop if not target is found
        }
    }

    @Override
    public boolean isFinished(){
        return llSub.hasTarget() && Math.abs(llSub.getTx()) < deadband;
    }

    @Override
    public void end(boolean interrupted){
        drive.drive(0,0,0);   //stop motors
    }
}
