package org.firstinspires.ftc.teamcode.Command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.limelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

public class rotateToTargetCMD extends CommandBase {
    private final mecanumDriveSubsystem drive;
    private final limelightSubsystem llSub;
    private final double kPRotation = Constants.kPRotation; //simple proportional gain
    private final double deadband = Constants.deadband; //degrees in which we stop
    public rotateToTargetCMD(mecanumDriveSubsystem drive, limelightSubsystem llSub){
        this.drive = drive;
        this.llSub = llSub;
        addRequirements(drive);
    }

    @Override
    public void execute(){
        if(llSub.hasTarget()) {
            double error = llSub.getTx(); //horizontal offset in degrees
            double rotPower = error * kPRotation;
            //clipped power to [-0.3, 0.3] for safety
            rotPower = Math.max(Math.min(rotPower, 0.3), -0.3);

            drive.drive(0, 0, rotPower); //rot only
        } else {
            drive.drive(0,0,0); //stop if no target is found
        }
    }

    @Override
    public boolean isFinished(){
        //if limelight returns with target and the Tx is less then the deadband the command will finish
        return llSub.hasTarget() && Math.abs(llSub.getTx()) < deadband;
    }

    @Override
    public void end(boolean interrupted){
        drive.drive(0,0,0);   //stop motors
    }
}
