package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class mecanumDriveSubsystem extends SubsystemBase {

    public final Motor m_Fl, m_Fr, m_Rl, m_Rr;
    public double fwdPower;
    public double strPower;
    public double rotPower;
    private final IMU imu;
    private final PIDController headingController;
    private double targetHeading;

    public mecanumDriveSubsystem (Motor front_left, Motor front_right,
                                  Motor back_left, Motor back_right, HardwareMap hardwareMap){
        m_Fl = front_left;
        m_Fr = front_right;
        m_Rl = back_left;
        m_Rr = back_right;

        //IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(parameters);

        //PID controller for heading hold
        //TODO: tune the gains
        headingController = new PIDController(1, 0, 0);
        targetHeading = getHeading();


    }

    public double getHeading(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /**
     * wraps a angle in radians to the range [-pi, pi]
     * This ensures that the angle differences are always measured
     * along the shortest path eg. [-181, 179]
     *----------------------------------------------------------------
     * This is super useful for PID heading control to prevent any
     * unusual behavior crossing the -pi, pi boundary
     */
    private double wrapAngle(double angle){
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public void drive(double forward, double strafe, double rotation,
                      boolean fieldCentric, boolean headingLock){
        fwdPower = forward;
        strPower = strafe;
        rotPower = rotation;

        // Field-centric transformation
        if(fieldCentric){
            double heading = getHeading();
            double temp = forward * Math.cos(heading) + strafe * Math.sin(heading);
            strafe = -forward * Math.sin(heading) + strafe * Math.cos(heading);
            forward = temp;
        }

        //heading lock only when the driver is NOT commanding rotation
        if(headingLock && Math.abs(rotation) < 0.05){
            double error = wrapAngle(targetHeading - getHeading());
            rotation = headingController.calculate(0, error);
        } else {
            targetHeading = getHeading();
        }

        double Fl = forward + strafe + rotation;
        double Fr = forward - strafe - rotation;
        double Rl = forward + strafe - rotation;
        double Rr = forward - strafe + rotation;

        //Normalize the powers
        double max = Math.max(1.0, Math.max(Math.abs(Fl),
                Math.max(Math.abs(Fr), Math.max(Math.abs(Rl), Math.abs(Rr)))));

        //set motor powers
        m_Fl.set(Fl / max);
        m_Fr.set(Fr / max);
        m_Rl.set(Rl / max);
        m_Rr.set(Rr / max);




    }

    //Power getters
    public double getFwdPower(){return fwdPower;}
    public double getStrPower(){return strPower;}
    public double getRotPower(){return rotPower;}

    //Motor getters
    public Motor getM_Fl() {return m_Fl;}
    public Motor getM_Fr() {return m_Fr;}
    public Motor getM_Rl() {return m_Rl;}
    public Motor getM_Rr() {return m_Rr;}
}
