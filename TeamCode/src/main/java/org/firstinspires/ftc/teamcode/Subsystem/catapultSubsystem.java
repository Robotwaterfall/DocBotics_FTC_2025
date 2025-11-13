package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class catapultSubsystem extends SubsystemBase {
    HardwareMap hardwareMap;

    private final DcMotor catapult1;
    private final DcMotor catapult2;

    private int cataSetpoint = 0;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();



    public catapultSubsystem(DcMotor m_catapult1, DcMotor m_catapult2){

        this.catapult1 = m_catapult1;
        this.catapult2 = m_catapult2;


        m_catapult1.setDirection(DcMotorSimple.Direction.REVERSE);
        m_catapult2.setDirection(DcMotorSimple.Direction.FORWARD);



    }

    public DcMotor getM_catapult1(){
        return catapult1;
    }

    public DcMotor getM_catapult2(){
        return catapult2;
    }

    public void setCataSetpoint(int setpoint){
        cataSetpoint = setpoint;
    }

    public int getCataSetpoint(){
        return cataSetpoint;
    }

    public void runToPosition(){
        catapult1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapult2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void stopAndRestEncoder(){
        catapult1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoder(){
        catapult1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        catapult2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void periodic() {
        // Send catapult telemetry to FTC Dashboard
        TelemetryPacket cataPacket = new TelemetryPacket();
        cataPacket.put("m_cataput1_motorpower", getM_catapult1().getPower());
        cataPacket.put("m_cataput2_motorpower", getM_catapult2().getPower());
        cataPacket.put("MotorticksM1", getM_catapult1().getCurrentPosition());
        cataPacket.put("MotorticksM2", getM_catapult2().getCurrentPosition());
        dashboard.sendTelemetryPacket(cataPacket);
    }
}
