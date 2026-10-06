package org.firstinspires.ftc.teamcode.teleop.competition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.RobotTeleop;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RelocalizePosition;
import org.firstinspires.ftc.teamcode.util.StartPosition;
import org.firstinspires.ftc.teamcode.util.Teleop;

@TeleOp(name = "Red Close Tele", group = "Competition")
public class RedCloseTeleop extends RobotTeleop {

    @Override
    protected RobotConfig getConfig() {
        return new RobotConfig(StartPosition.RED, Alliance.RED, Teleop.TRUE, RelocalizePosition.RED_CORNER);
    }
}