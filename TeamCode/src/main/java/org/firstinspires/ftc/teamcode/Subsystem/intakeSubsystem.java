package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class intakeSubsystem extends SubsystemBase {

    DcMotor m_intakeMotor;
    boolean intakeRunning = false;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public intakeSubsystem(DcMotor intakeMotor){
        m_intakeMotor = intakeMotor;

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE); //TODO: check intake motor direction
    }

    public void setM_intakeMotorPower(double power){
        intakeRunning = true;
        m_intakeMotor.setPower(power);
    }

    public boolean isIntakeRunning(){
        return intakeRunning;
    }

    @Override
    public void periodic() {
        TelemetryPacket intakePacket = new TelemetryPacket();
        intakePacket.put("intakeRunning: ", isIntakeRunning());
        dashboard.sendTelemetryPacket(intakePacket);

    }
}
