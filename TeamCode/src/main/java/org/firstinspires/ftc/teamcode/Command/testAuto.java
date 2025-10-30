package org.firstinspires.ftc.teamcode.Command;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

@TeleOp(name = "testAuto")
public class testAuto extends OpMode {
    mecanumDriveSubsystem driveSub;
    GamepadEx gamepad;


    @Override
    public void init() {
        driveSub.resetDriveEncoders();
    }

    @Override
    public void loop() {

        if(gamepad.isDown(GamepadKeys.Button.A)) {
            driveSub.xCordinate(8); // 8 inches forward
        } else {
            driveSub.drive(0,0,0); // stop all motors if a is not pressed
        }
    }


}
