package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class catapultSubsystem extends SubsystemBase {
    private final DcMotor m_catapult1, m_catapult2;
    HardwareMap hardwareMap;

    public static enum CatapultModes {UP, DOWN, BRAKE}
    public static CatapultModes pivotMode;


    public catapultSubsystem(DcMotor m_catapult1, DcMotor m_catapult2){

        this.m_catapult1 = m_catapult1;
        this.m_catapult2 = m_catapult2;

        m_catapult1.setDirection(DcMotorSimple.Direction.REVERSE);
        m_catapult2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public DcMotor getM_catapult1(){
        return m_catapult1;
    }

    public DcMotor getM_catapult2(){
        return m_catapult2;
    }

    public void stopMotors(){
        m_catapult1.setPower(0);
        m_catapult2.setPower(0);
    }






}
