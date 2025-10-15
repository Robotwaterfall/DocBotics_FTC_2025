package org.firstinspires.ftc.teamcode.Command;

import static org.firstinspires.ftc.teamcode.Constants.kPRotation;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.limelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

import java.util.function.Supplier;

public class teleOpMecanumDriveCommand extends CommandBase {


    private final mecanumDriveSubsystem driveSub;
    private final limelightSubsystem llsub;
    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;
    private final Supplier<Double> rSupplier;

    public teleOpMecanumDriveCommand(
            mecanumDriveSubsystem driveSub,
            limelightSubsystem llsub,
            Supplier<Double> xSupplier,
            Supplier<Double> ySupplier,
            Supplier<Double> rSupplier
            ) {

        this.driveSub = driveSub;
        this.llsub = llsub;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rSupplier = rSupplier;
        addRequirements(driveSub);
    }

    @Override
    public void execute() {
        if(llsub.hasTarget()) {
            //gets the joystick values
            double forward = ySupplier.get();
            double strafe  = xSupplier.get();

            double error = llsub.getTx(); //horizontal offset in degrees
            double rotPower = error * kPRotation;

            //clipped power to [-0.4, 0.4] for safety
            rotPower = Math.max(Math.min(rotPower, 0.4), -0.4);

            double strPower = rotPower;

            driveSub.drive(forward, strafe, rotPower); //rot only
        } else {
            //Normal teleoperated drive if limelight does not see april tag
            double forward = ySupplier.get();
            double strafe  = xSupplier.get();
            double rotation = rSupplier.get();

            driveSub.drive(forward, strafe, rotation);
        }

    }
}