package org.firstinspires.ftc.teamcode.Auto.Paths;

import static org.firstinspires.ftc.teamcode.Constants.cata_Down_setpoint;
import static org.firstinspires.ftc.teamcode.Constants.cata_Up_setpoint;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Auto.MoveRobotEncoderXY_CMD;
import org.firstinspires.ftc.teamcode.Auto.autoRobotContainer;
import org.firstinspires.ftc.teamcode.Command.catapultCommand;

@Disabled
@Autonomous
public class Threeartificat_FarLeft_shoot extends autoRobotContainer {

    @Override
    public void path() {

        schedule(new SequentialCommandGroup(
                new MoveRobotEncoderXY_CMD(20,20,3,0.7,driveSub),
                new MoveRobotEncoderXY_CMD(30,-30,2,0.7,driveSub),
                new MoveRobotEncoderXY_CMD(25,25,3, 0.7, driveSub),
                new SequentialCommandGroup(
                        new catapultCommand(cataSub, cata_Down_setpoint),
                        new catapultCommand(cataSub, cata_Up_setpoint)
                )
        ));
    }
}
