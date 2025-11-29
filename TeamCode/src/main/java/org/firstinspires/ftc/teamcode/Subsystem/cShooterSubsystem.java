package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class cShooterSubsystem extends SubsystemBase {

    DcMotor shooterMotor1;
    DcMotor shooterMotor2;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public cShooterSubsystem(DcMotor m_shooterMotor1, DcMotor m_shooterMotor2){
        this.shooterMotor1 = m_shooterMotor1;
        this.shooterMotor2 = m_shooterMotor2;

        shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void setShooterMotorPower(double motorPower){
        shooterMotor1.setPower(motorPower);
        shooterMotor2.setPower(motorPower);
    }

    public double getMotorPower(){
        return shooterMotor1.getPower();
    }

    @Override
    public void periodic() {
        TelemetryPacket shooterPacket = new TelemetryPacket();
        shooterPacket.put("shooterMotorPower: ", getMotorPower());
        dashboard.sendTelemetryPacket(shooterPacket);
    }
}
