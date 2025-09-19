package org.firstinspires.ftc.teamcode.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

import java.util.function.Supplier;

public class teleOpMecanumDriveCommand extends CommandBase {

    private final mecanumDriveSubsystem driveSub;
    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;
    private final Supplier<Double> rSupplier;
    private final Supplier<Boolean> fieldCentricSupplier; // Optional toggle

    public teleOpMecanumDriveCommand(
            mecanumDriveSubsystem driveSub,
            Supplier<Double> xSupplier,
            Supplier<Double> ySupplier,
            Supplier<Double> rSupplier,
            Supplier<Boolean> fieldCentricSupplier) {

        this.driveSub = driveSub;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rSupplier = rSupplier;
        this.fieldCentricSupplier = fieldCentricSupplier;
        addRequirements(driveSub);
    }

    @Override
    public void execute() {
        // Read joystick inputs
        double forward = ySupplier.get();
        double strafe  = xSupplier.get();
        double rotation = rSupplier.get();

        // Field-centric toggle
        boolean fieldCentric = fieldCentricSupplier.get();

        // Pass values to the subsystem drive method
        driveSub.drive(forward, strafe, rotation, fieldCentric);
    }
}