package org.firstinspires.ftc.teamcode.AutoCommands;

import static org.firstinspires.ftc.teamcode.Constants.angularMaxSpeedError;
import static org.firstinspires.ftc.teamcode.Constants.angularSpeedController_kd;
import static org.firstinspires.ftc.teamcode.Constants.angularSpeedController_kp;
import static org.firstinspires.ftc.teamcode.Constants.drivetrain_ka_y;
import static org.firstinspires.ftc.teamcode.Constants.drivetrain_ks_y;
import static org.firstinspires.ftc.teamcode.Constants.drivetrain_kv_y;
import static org.firstinspires.ftc.teamcode.Constants.linearMaxSpeedError;
import static org.firstinspires.ftc.teamcode.Constants.drivetrain_ka_x;
import static org.firstinspires.ftc.teamcode.Constants.drivetrain_ks_x;
import static org.firstinspires.ftc.teamcode.Constants.drivetrain_kv_x;
import static org.firstinspires.ftc.teamcode.Constants.xSpeedController_kd;
import static org.firstinspires.ftc.teamcode.Constants.xSpeedController_kp;
import static org.firstinspires.ftc.teamcode.Constants.ySpeedController_kd;
import static org.firstinspires.ftc.teamcode.Constants.ySpeedController_kp;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

public class accelerateToSpeedCommand extends CommandBase {

    private final mecanumDriveSubsystem driveSub;

    private final double desiredSpeed_x_inchPerSec;
    private final double desiredSpeed_y_inchPerSec;
    private final double desiredSpeed_angle_degreesPerSec;



    private final PIDController xSpeedControler;
    private final PIDController ySpeedControler;
    private final PIDController hSpeedControler;

    public accelerateToSpeedCommand(
            mecanumDriveSubsystem driveSub,
            double desiredSpeed_x_inchPerSec,
            double desiredSpeed_y_inchPerSec,
            double desiredSpeed_angle_degreesPerSec



) {     //initializes the x Speed Controler PID Constants and error tolerance.
        this.xSpeedControler = new PIDController(xSpeedController_kp,0, xSpeedController_kd);
        this.xSpeedControler.setTolerance(linearMaxSpeedError);
        //initializes the y Speed Controller PID Constants and error tolerance.
        this.ySpeedControler = new PIDController(ySpeedController_kp,0, ySpeedController_kd);
        this.ySpeedControler.setTolerance(linearMaxSpeedError);

        //initializes the h Speed Controller PID Constants and error tolerance.
        this.hSpeedControler = new PIDController(angularSpeedController_kp,0, angularSpeedController_kd);
        this.hSpeedControler.setTolerance(angularMaxSpeedError);

        this.desiredSpeed_x_inchPerSec = desiredSpeed_x_inchPerSec;
        this.desiredSpeed_y_inchPerSec = desiredSpeed_y_inchPerSec;
        this.desiredSpeed_angle_degreesPerSec = desiredSpeed_angle_degreesPerSec;
        this.driveSub = driveSub;

        addRequirements(driveSub);
    }

    @Override
    public void execute() {
        //update SOOS speed measurements.
        double currentRobotSpeed_x_inchesPerSec = driveSub.SOOS_Velocity_x_inchesPerSec;
        double currentRobotSpeed_y_inchesPerSec = driveSub.SOOS_Velocity_y_inchesPerSec;
        double currentRobotSpeed_angle_degreesPerSec = driveSub.SOOS_Velocity_h_degreesPerSec;


        //updates the PID outputs for the speed controllers.
        double xSpeedPID_Output = xSpeedControler.calculate(currentRobotSpeed_x_inchesPerSec, desiredSpeed_x_inchPerSec);
        double ySpeedPID_Output= ySpeedControler.calculate(currentRobotSpeed_y_inchesPerSec, desiredSpeed_y_inchPerSec);
        double hSpeedPID_Output = hSpeedControler.calculate(currentRobotSpeed_angle_degreesPerSec, desiredSpeed_angle_degreesPerSec);

        //updates the Feedforward outputs for the speed controllers.
        double feedforwardController_x = drivetrain_ks_x
                + (Math.abs(driveSub.SOOS_Velocity_x_inchesPerSec) * drivetrain_kv_x)
                + (Math.abs(driveSub.SOOS_Acceleration_x_inchesPerSecSqaured) * drivetrain_ka_x);

        double feedforwardController_y = drivetrain_ks_y
                + (Math.abs(driveSub.SOOS_Velocity_y_inchesPerSec) * drivetrain_kv_y)
                + (Math.abs(driveSub.SOOS_Acceleration_y_inchesPerSecSquared) * drivetrain_ka_y);

        double feedForwardCotnroller_h = 0;

        double xOutput = (Math.signum(xSpeedPID_Output)* feedforwardController_x) + xSpeedPID_Output;
        double yOutput = (Math.signum(ySpeedPID_Output)*feedforwardController_y )+ ySpeedPID_Output;
        double hOutput = (Math.signum(hSpeedPID_Output)*feedForwardCotnroller_h )+ hSpeedPID_Output;
        driveSub.drive(xOutput, yOutput,  hOutput,false);


    }
    @Override
    public boolean isFinished() {

        return  xSpeedControler.atSetPoint() &&
                ySpeedControler.atSetPoint() &&
                hSpeedControler.atSetPoint();
    }

}