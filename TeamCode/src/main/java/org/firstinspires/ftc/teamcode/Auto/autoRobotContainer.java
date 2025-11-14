package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.catapultSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

public class autoRobotContainer extends CommandOpMode {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public mecanumDriveSubsystem driveSub;


    public catapultSubsystem cataSub;
    public DcMotor cataMotor;
    public DcMotor cata2Motor;




    @Override
    public void initialize() {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");


        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        cataSub = new catapultSubsystem(
               cataMotor = hardwareMap.get(DcMotor.class, "CatapultMotor1"),
               cata2Motor = hardwareMap.get(DcMotor.class, "CatapultMotor2")
        );

        cataMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        cata2Motor.setDirection(DcMotorSimple.Direction.FORWARD);





        initSubsystems();
        path();


    }


    private void initSubsystems(){
        driveSub = new mecanumDriveSubsystem(frontLeft, frontRight, backLeft, backRight, hardwareMap);

        cataSub = new catapultSubsystem(cataMotor, cata2Motor);

    }

    public void path(){

    }
}
