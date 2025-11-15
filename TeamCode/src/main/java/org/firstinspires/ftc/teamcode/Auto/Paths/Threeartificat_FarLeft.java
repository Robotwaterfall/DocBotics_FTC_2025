package org.firstinspires.ftc.teamcode.Auto.Paths;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Auto.MoveRobotEncoderXY_CMD;
import org.firstinspires.ftc.teamcode.Auto.autoRobotContainer;

@Disabled
@Autonomous
public class Threeartificat_FarLeft extends autoRobotContainer {

    @Override
    public void path() {

        schedule(new SequentialCommandGroup(
                new MoveRobotEncoderXY_CMD(20,20,3,0.8,driveSub),
                new MoveRobotEncoderXY_CMD(30,-30,2,0.8,driveSub),
                new MoveRobotEncoderXY_CMD(25,25,3, 0.8, driveSub)
        ));
    }
}
