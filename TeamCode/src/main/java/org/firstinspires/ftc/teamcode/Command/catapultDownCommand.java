package org.firstinspires.ftc.teamcode.Command;

import static org.firstinspires.ftc.teamcode.Constants.cataTimeElapsed;
import static org.firstinspires.ftc.teamcode.Constants.catapult_down_power;
import static org.firstinspires.ftc.teamcode.Subsystem.catapultSubsystem.pivotMode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.catapultSubsystem;

public class catapultDownCommand extends CommandBase {
        private catapultSubsystem cataSub;
        private ElapsedTime cataDownTime = new ElapsedTime();

        public catapultDownCommand(catapultSubsystem cataSub){
            this.cataSub = cataSub;
        }

        @Override
        public void initialize(){

            //when initialized catapult starts with motor brakes on
            cataSub.getM_catapult1().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            cataSub.getM_catapult2().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //resets the catapults run time
            cataDownTime.reset();

        }

        @Override
        public void execute(){
            pivotMode = catapultSubsystem.CatapultModes.DOWN;

            cataDownTime.startTime();


            while(cataDownTime.seconds() < cataTimeElapsed){

                pivotMode = catapultSubsystem.CatapultModes.DOWN;
                //power the motors to make the catapult go back down
                cataSub.getM_catapult1().setPower(catapult_down_power);
                cataSub.getM_catapult2().setPower(catapult_down_power);
            }

            pivotMode = catapultSubsystem.CatapultModes.BRAKE;
            cataSub.getM_catapult1().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            cataSub.getM_catapult1().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        }
    }

