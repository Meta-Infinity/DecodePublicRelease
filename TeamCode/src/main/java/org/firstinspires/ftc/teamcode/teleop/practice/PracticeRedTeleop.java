package org.firstinspires.ftc.teamcode.teleop.practice;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.RobotTeleop;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RelocalizePosition;
import org.firstinspires.ftc.teamcode.util.StartPosition;
import org.firstinspires.ftc.teamcode.util.Teleop;

@TeleOp(name = "Practice Red Tele", group = "Practice")
public class PracticeRedTeleop extends RobotTeleop {

    @Override
    protected RobotConfig getConfig() {
        return new RobotConfig(StartPosition.CENTER, Alliance.RED, Teleop.TRUE, RelocalizePosition.RED_CORNER);
    }
}