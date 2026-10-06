package org.firstinspires.ftc.teamcode.teleop.competition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.RobotTeleop;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RelocalizePosition;
import org.firstinspires.ftc.teamcode.util.StartPosition;
import org.firstinspires.ftc.teamcode.util.Teleop;

@TeleOp(name = "Blue Close Tele", group = "Competition")
public class BlueCloseTeleop extends RobotTeleop {

    @Override
    protected RobotConfig getConfig() {
        return new RobotConfig(StartPosition.BLUE, Alliance.BLUE, Teleop.TRUE, RelocalizePosition.BLUE_CORNER);
    }
}