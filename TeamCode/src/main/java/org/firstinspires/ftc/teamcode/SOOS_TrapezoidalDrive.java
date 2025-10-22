package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.drivetrain_ka;
import static org.firstinspires.ftc.teamcode.Constants.drivetrain_ks;
import static org.firstinspires.ftc.teamcode.Constants.drivetrain_kv;
import static org.firstinspires.ftc.teamcode.Constants.drivetrain_maxAccel;
import static org.firstinspires.ftc.teamcode.Constants.drivetrain_maxVelocity;
import static org.firstinspires.ftc.teamcode.Constants.drivetrain_targetDistance;
import static org.firstinspires.ftc.teamcode.Constants.heading_Kd;
import static org.firstinspires.ftc.teamcode.Constants.heading_Ki;
import static org.firstinspires.ftc.teamcode.Constants.heading_Kp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

@Autonomous (name = "SOOS_TrapezoidalDrive")
public class SOOS_TrapezoidalDrive extends OpMode {
    private Motor front_left;
    private Motor front_right;
    private Motor back_left;
    private Motor back_right;

    SparkFunOTOS myOtos;

    IMU imu;
    IMU.Parameters myIMUParameters;


    TelemetryPacket packet = new TelemetryPacket();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    // Motion profile parameters
    private final double maxAccel = drivetrain_maxAccel; // inches/sec^2
    private final double maxVel = drivetrain_maxVelocity;   // inches/sec
    private final double targetDistance = drivetrain_targetDistance; // inches

    // Feedforward Constants
    private final double Kv = drivetrain_kv;
    private final double Ka = drivetrain_ka;
    private final double Ks = drivetrain_ks;

    // HeadingPID constants
    private final double Kp = heading_Kp;
    private final double Ki = heading_Ki;
    private final double Kd = heading_Kd;

    private double lastHeadingError = 0;
    private double headingIntegral = 0;
    private double lastHeadingTime = 0;

    //times
    private ElapsedTime timer = new ElapsedTime();
    private double totalTime;
    private double accelTime;
    private double cruiseTime;

    //Current phase
    private enum Phase { ACCEL, CRUISE, DECEL, DONE }
    private Phase currentPhase = Phase.ACCEL;


    @Override
    public void init(){
        //assigning name
        imu = hardwareMap.get(IMU.class, "imu");
        myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu.resetYaw();  // sets heading to zero at start

        //assigning name to motors
        front_left = new Motor(hardwareMap, "front_left");
        front_right = new Motor(hardwareMap, "front_right");
        back_left = new Motor(hardwareMap, "back_left");
        back_right = new Motor(hardwareMap, "back_right");

        //assigning name
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        // Calculate profile timing once
        accelTime = maxVel / maxAccel;
        double accelDist = 0.5 * maxAccel * accelTime * accelTime;
        if (2 * accelDist > targetDistance) {
            accelTime = Math.sqrt(targetDistance / maxAccel);
        }
        double cruiseDist = targetDistance - 2 * (0.5 * maxAccel * accelTime * accelTime);
        cruiseTime = cruiseDist / maxVel;
        totalTime = 2 * accelTime + cruiseTime;
        //reset timer
        timer.reset();

        //Configure SOOS at startup
        configureOtos();
    }


    @Override
    public void loop() {
        double t = timer.seconds();
        double desiredVel = 0;
        double desiredAccel = 0;

        // --- Trapezoid calculation ---
        if (t < accelTime) { // accelerating
            desiredAccel = maxAccel;
            desiredVel = maxAccel * t;
            currentPhase = Phase.ACCEL;
        } else if (t < accelTime + cruiseTime) { // cruising
            desiredAccel = 0;
            desiredVel = maxVel;
            currentPhase = Phase.CRUISE;
        } else if (t < totalTime) { // decelerating
            desiredAccel = -maxAccel;
            double timeSinceDecel = t - (accelTime + cruiseTime);
            desiredVel = maxVel - maxAccel * timeSinceDecel;
            currentPhase = Phase.DECEL;
        } else { // done
            desiredAccel = 0;
            desiredVel = 0;
            currentPhase = Phase.DONE;
        }

        // --- Feedforward motor power ---
        double feedforward = Ks + Kv * desiredVel + Ka * desiredAccel;

        // Stop once the sensor reads we've moved ~targetDistance
        double distanceTraveled = Math.hypot(myOtos.getPosition().x, myOtos.getPosition().y);
        if (distanceTraveled >= targetDistance) {
            feedforward = 0;
            currentPhase = Phase.DONE;
        }

        //get current heading
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        //Desired heading straight is 0
        double headingError = 0 - currentHeading;
        double currentTime = timer.seconds();
        double dt = currentTime - lastHeadingTime;

        //Heading PID
        headingIntegral += headingError * dt;
        double headingDerivative = (headingError - lastHeadingError) / dt;
        double headingCorrection = heading_Kp * headingError + heading_Ki * headingIntegral + heading_Kd * headingDerivative;

        // Save states
        lastHeadingError = headingError;
        lastHeadingTime = currentTime;


        double leftPower = feedforward - headingCorrection;
        double rightPower = feedforward + headingCorrection;

        //clamp power max power to 100%
        leftPower = Math.max(-1, Math.min(1, leftPower));
        rightPower = Math.max(-1, Math.min(1, rightPower));


        //set powers to motors
        front_left.set(leftPower);
        front_right.set(rightPower);
        back_left.set(leftPower);
        back_right.set(rightPower);

        // --- Telemetry ---
        packet.put("Phase", currentPhase);
        packet.put("Time", t);
        packet.put("Desired Velocity", desiredVel);
        packet.put("Desired Accel", desiredAccel);
        packet.put("Feedforward", feedforward);
        packet.put("Distance", distanceTraveled);
        dashboard.sendTelemetryPacket(packet);
    }


    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

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
        // -y right +y forward

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

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }




}
