package org.firstinspires.ftc.teamcode.Command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.limelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

public class drive_straight_to_distanceCMD extends CommandBase {

    private final mecanumDriveSubsystem driveSub;
    private final limelightSubsystem llSub;
    private final double targetDistance; //meters

    private final double kPForward = Constants.kPForward;
    private final double kPRotate = Constants.kPRotation;
    private final double tolerance = Constants.tolerance;

    public drive_straight_to_distanceCMD(mecanumDriveSubsystem driveSub, limelightSubsystem llSub,
                                         double targetDistanceMeters) {
        this. driveSub = driveSub;
        this. llSub = llSub;
        this. targetDistance = targetDistanceMeters;
        addRequirements(driveSub);
    }

    @Override
    public void execute(){
        if (llSub.hasTarget()){
            // Forward control using ty (distance from limelight to target)
            double distanceError = llSub.getDistance() - targetDistance;
            double fwdPower = distanceError * kPForward;

            // Rotational control using tx (horizontal offset of limelight to target)
            double tx = llSub.getTx();
            double rotPower = -tx * kPRotate;

            // Safety limits
            fwdPower = Math.max(Math.min(fwdPower, 0.4), -0.4);
            rotPower = Math.max(Math.min(rotPower, 0.3), -0.3);
        } else {
            driveSub.drive(0,0,0);
        }
    }

    @Override
    public boolean isFinished(){
        return llSub.hasTarget()
                && Math.abs(llSub.getDistance() - targetDistance) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        driveSub.drive(0, 0, 0);
    }
}
