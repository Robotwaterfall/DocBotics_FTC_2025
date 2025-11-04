package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Subsystem.catapultSubsystem.pivotMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystem.catapultSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

public class TelemetryManager {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    catapultSubsystem cataSub;
    intakeSubsystem intakeSub;
    mecanumDriveSubsystem driveSub;

    // Send telemetry to FTC Dashboard

    public void runTelemetry() {

        // Send drive telemetry to FTC Dashboard
        TelemetryPacket drivePacket = new TelemetryPacket();
        drivePacket.put("Forward", driveSub.getFwdPower());
        drivePacket.put("Strafe", driveSub.getStrPower());
        drivePacket.put("Rotation", driveSub.getRotPower());
        drivePacket.put("FL Power", driveSub.getFl());
        drivePacket.put("FR Power", driveSub.getFr());
        drivePacket.put("BL Power", driveSub.getRl());
        drivePacket.put("BR Power", driveSub.getRr());
        dashboard.sendTelemetryPacket(drivePacket);

        // Send intake telemetry to FTC Dashboard
        TelemetryPacket intakePacket = new TelemetryPacket();
        intakePacket.put("intakeRunning: ", intakeSub.isIntakeRunning());
        dashboard.sendTelemetryPacket(intakePacket);

        // Send catapult telemetry to FTC Dashboard
        TelemetryPacket cataPacket = new TelemetryPacket();
        cataPacket.put("cataPhase", pivotMode);
        cataPacket.put("m_cataput1_motorpower", cataSub.getM_catapult1().getPower());
        cataPacket.put("m_cataput2_motorpower", cataSub.getM_catapult2().getPower());
        dashboard.sendTelemetryPacket(cataPacket);
    }

}
