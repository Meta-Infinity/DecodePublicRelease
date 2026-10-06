package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RelocalizePosition;
import org.firstinspires.ftc.teamcode.util.StartPosition;
import org.firstinspires.ftc.teamcode.util.Teleop;

@Config
public class RobotConfig {
    public static double RED_ENCODER_WRAP_OFFSET_DEG = 180;
    public static double BLUE_ENCODER_WRAP_OFFSET_DEG = 180;

    public final Pose startPose;
    public final Alliance alliance;
    public final StartPosition startPosition;
    public final Teleop teleop;
    public final RelocalizePosition relocalizePosition;
    public final Pose relocalizePose;

    public RobotConfig(StartPosition startPosition, Alliance alliance, Teleop teleop, RelocalizePosition relocalizePosition) {
        this.startPosition = startPosition;
        this.startPose = startPosition.defaultPose;
        this.alliance = alliance;
        this.teleop = teleop;
        this.relocalizePose = relocalizePosition.defaultPose;
        this.relocalizePosition = relocalizePosition;
    }

    public static double getEncoderWrapOffsetDeg(Alliance alliance) {
        return alliance == Alliance.RED ? RED_ENCODER_WRAP_OFFSET_DEG : BLUE_ENCODER_WRAP_OFFSET_DEG;
    }
}
