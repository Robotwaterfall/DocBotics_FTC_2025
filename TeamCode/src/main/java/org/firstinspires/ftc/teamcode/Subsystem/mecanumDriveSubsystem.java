package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class mecanumDriveSubsystem extends SubsystemBase {

    private final Motor m_Fl, m_Fr, m_Rl, m_Rr;

    // Store last joystick values for telemetry
    private double fwdPower, strPower, rotPower;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public mecanumDriveSubsystem(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight, HardwareMap hardwareMap) {
        m_Fl = frontLeft;
        m_Fr = frontRight;
        m_Rl = backLeft;
        m_Rr = backRight;

        // Set motor directions (typical mecanum setup)
        m_Fl.setInverted(false);
        m_Fr.setInverted(true);
        m_Rl.setInverted(false);
        m_Rr.setInverted(true);
    }

    /**
     * Drive method for teleop.
     * @param forward forward/backward input (-1 to 1)
     * @param strafe left/right input (-1 to 1)
     * @param rotation rotation input (-1 to 1)
     * @param fieldCentric whether to use field-centric control
     */
    public void drive(double forward, double strafe, double rotation, boolean fieldCentric) {
        // Apply deadzone
        forward = applyDeadzone(forward, 0.05);
        strafe  = applyDeadzone(strafe, 0.05);
        rotation = applyDeadzone(rotation, 0.05);

        fwdPower = forward;
        strPower = strafe;
        rotPower = rotation;

        // Field-centric transform
        if (fieldCentric) {
            double heading = -getHeading(); // Negate if needed
            double temp = forward * Math.cos(heading) + strafe * Math.sin(heading);
            strafe = -forward * Math.sin(heading) + strafe * Math.cos(heading);
            forward = temp;
        }

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
        m_Fl.set(fl);
        m_Fr.set(fr);
        m_Rl.set(bl);
        m_Rr.set(br);
    }

    // Deadzone helper
    private double applyDeadzone(double value, double threshold) {
        return Math.abs(value) > threshold ? value : 0;
    }

    // --- Placeholder heading method ---
    // Replace this with your IMU code
    public double getHeading() {
        // Return robot heading in radians
        // Example: imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return 0.0;
    }

    @Override
    public void periodic() {
        // Send telemetry to FTC Dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Forward", fwdPower);
        packet.put("Strafe", strPower);
        packet.put("Rotation", rotPower);
        packet.put("FL Power", m_Fl.get());
        packet.put("FR Power", m_Fr.get());
        packet.put("BL Power", m_Rl.get());
        packet.put("BR Power", m_Rr.get());
        packet.put("Heading (deg)", Math.toDegrees(getHeading()));
        dashboard.sendTelemetryPacket(packet);
    }
}