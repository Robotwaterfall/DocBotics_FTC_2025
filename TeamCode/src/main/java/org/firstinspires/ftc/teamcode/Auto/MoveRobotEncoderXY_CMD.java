package org.firstinspires.ftc.teamcode.Auto;



import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

public class MoveRobotEncoderXY_CMD extends CommandBase {
    private final mecanumDriveSubsystem m_MecanumDriveSubsystem;

    private final double m_leftInches;
    private final double m_rightInches;

    private final double m_timeoutS;
    private final double m_speed;
    private DcMotor frontLeft;
    private DcMotor frontRight;

    private DcMotor backLeft;
    private DcMotor backRight;
    private final ElapsedTime runtime = new ElapsedTime();
    public MoveRobotEncoderXY_CMD(double leftInches,
                                  double rightInches,
                                  double timeoutS,
                                  double speed,
                                  mecanumDriveSubsystem driveSubsystem) {
        m_leftInches = leftInches;
        m_rightInches = rightInches;
        m_timeoutS = timeoutS;
        m_speed = speed;

        m_MecanumDriveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {

        frontLeft = m_MecanumDriveSubsystem.m_Fl;
        frontRight = m_MecanumDriveSubsystem.m_Fr;
        backLeft = m_MecanumDriveSubsystem.m_Rl;
        backRight = m_MecanumDriveSubsystem.m_Rr;

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;




        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(-m_leftInches * Constants.encoderAutoConstants.COUNTS_PER_INCH);
        newFrontRightTarget = frontRight.getCurrentPosition() + (int)(-m_rightInches * Constants.encoderAutoConstants.COUNTS_PER_INCH);

        newBackLeftTarget = backLeft.getCurrentPosition() + ((int)(-m_leftInches * Constants.encoderAutoConstants.COUNTS_PER_INCH));
        newBackRightTarget = backRight.getCurrentPosition() + ((int)(-m_rightInches * Constants.encoderAutoConstants.COUNTS_PER_INCH));


        frontLeft.setTargetPosition(newFrontLeftTarget);
        frontRight.setTargetPosition(-newFrontRightTarget);

        backLeft.setTargetPosition(-newBackLeftTarget);
        backRight.setTargetPosition(newBackRightTarget);




        // Turn On RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
    }

    @Override
    public void execute() {
        frontLeft.setPower(Math.abs(m_speed));
        backLeft.setPower(Math.abs(m_speed));
//
        //responsible for powering motors
        frontRight.setPower(Math.abs(m_speed));
        backRight.setPower(Math.abs(m_speed));

    }

    @Override
    public boolean isFinished(){
        if((runtime.seconds() < m_timeoutS))
        {
            return false;
        }
        if(frontRight.isBusy()  || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy())
        {
            return false;
        }


        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Turn off RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return true;



    }


}