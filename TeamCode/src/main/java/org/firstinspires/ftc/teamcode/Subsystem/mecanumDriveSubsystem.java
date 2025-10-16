package org.firstinspires.ftc.teamcode.Subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class mecanumDriveSubsystem extends SubsystemBase {

    private final Motor m_Fl, m_Fr, m_Rl, m_Rr;
    private final SparkFunOTOS myOtos;

    // Store last joystick values for telemetry
    private double fwdPower, strPower, rotPower;


    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public mecanumDriveSubsystem(Motor frontLeft, Motor frontRight, Motor backLeft,
                                 Motor backRight, SparkFunOTOS otos) {

        this.myOtos = otos;
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
        packet.put("Heading (deg) IMU", Math.toDegrees(getHeading()));



        //SOOS is rotated 90 degrees from recomend configuration
        // so readings for x and y need to be swapped
        packet.put("SOOS X INCHES", myOtos.getPosition().y * Constants.soos_Linear_scaler );
        packet.put("SOOS Y INCHES", myOtos.getPosition().x * Constants.soos_Linear_scaler );
        packet.put("SOOS angle Degrees ", myOtos.getPosition().h *Constants.soos_Angular_scaler);
        packet.put("SOOS Linear scaler ", Constants.soos_Linear_scaler );
        // + counter clockwise, - clockwise
        packet.put("SOOS angular scaler ", Constants.soos_Angular_scaler);


        dashboard.sendTelemetryPacket(packet);
    }
    private void configureOtos() {



        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).

        // REF : RobotCentric
        // -x left +x Right
        // +y back -y forward

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-5.53, -5.03, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1);
        myOtos.setAngularScalar(1);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

    }
    //*resets the position readings on OTOS
    public void resetOtosPos(){
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);
    }

}