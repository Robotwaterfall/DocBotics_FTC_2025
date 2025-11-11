package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class mecanumDriveSubsystem extends SubsystemBase {

    private final DcMotor m_Fl, m_Fr, m_Rl, m_Rr;

    // Store last joystick values for telemetry
    private double fwdPower, strPower, rotPower;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public mecanumDriveSubsystem(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, HardwareMap hardwareMap) {
        m_Fl = frontLeft;
        m_Fr = frontRight;
        m_Rl = backLeft;
        m_Rr = backRight;

        // Set motor directions (typical mecanum setup)
        m_Fl.setDirection(DcMotorSimple.Direction.FORWARD);
        m_Fr.setDirection(DcMotorSimple.Direction.REVERSE);
        m_Rl.setDirection(DcMotorSimple.Direction.FORWARD);
        m_Rr.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Drive method for teleop.
     * @param forward forward/backward input (-1 to 1)
     * @param strafe left/right input (-1 to 1)
     * @param rotation rotation input (-1 to 1)
     */
    public void drive(double forward, double strafe, double rotation) {
        // Apply deadzone
        forward = applyDeadzone(forward, 0.05);
        strafe  = applyDeadzone(strafe, 0.05);
        rotation = applyDeadzone(rotation, 0.05);

        fwdPower = forward;
        strPower = strafe;
        rotPower = rotation;


        // Mecanum kinematics
        double fl = forward + strafe + rotation;
        double fr = forward - strafe + rotation;
        double bl = forward - strafe - rotation;
        double br = forward + strafe - rotation;

        // Normalize motor powers
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        // Set motor powers
        m_Fl.setPower(fl);
        m_Fr.setPower(fr);
        m_Rl.setPower(bl);
        m_Rr.setPower(br);
    }

    // Deadzone helper
    private double applyDeadzone(double value, double threshold) {
        return Math.abs(value) > threshold ? value : 0;
    }

    public DcMotor getFl(){
        return m_Fl;
    }
    public DcMotor getFr(){
        return m_Fr;
    }
    public DcMotor getRl(){
        return m_Rl;
    }
    public DcMotor getRr(){
        return m_Rr;
    }

    public double getFwdPower(){
        return fwdPower;
    }

    public double getStrPower(){
        return strPower;
    }

    public double getRotPower(){
        return rotPower;
    }

    @Override
    public void periodic() {
        TelemetryPacket drivePacket = new TelemetryPacket();
        drivePacket.put("Forward", getFwdPower());
        drivePacket.put("Strafe", getStrPower());
        drivePacket.put("Rotation", getRotPower());
        drivePacket.put("FL Power", getFl());
        drivePacket.put("FR Power", getFr());
        drivePacket.put("BL Power", getRl());
        drivePacket.put("BR Power", getRr());
        dashboard.sendTelemetryPacket(drivePacket);
    }
}