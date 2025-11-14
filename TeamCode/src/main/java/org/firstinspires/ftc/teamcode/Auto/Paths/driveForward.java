package org.firstinspires.ftc.teamcode.Auto.Paths;

import static org.firstinspires.ftc.teamcode.Constants.cata_Down_setpoint;
import static org.firstinspires.ftc.teamcode.Constants.cata_Up_setpoint;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.MoveRobotEncoderXY_CMD;
import org.firstinspires.ftc.teamcode.Auto.autoRobotContainer;
import org.firstinspires.ftc.teamcode.Auto.waitCommand;
import org.firstinspires.ftc.teamcode.Command.catapultCommand;

@Autonomous
public class driveForward extends autoRobotContainer {

    @Override
    public void path() {

        schedule(new SequentialCommandGroup(

                new MoveRobotEncoderXY_CMD(15,15,3,0.4, driveSub),
                new MoveRobotEncoderXY_CMD(15,-15,2,0.4,driveSub),
                new SequentialCommandGroup(
                        new catapultCommand(cataSub, cata_Up_setpoint),
                        new catapultCommand(cataSub, cata_Down_setpoint)
                )

        ));
    }
}
