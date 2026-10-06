package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public class FieldConstants {

    public static Pose getGoalPose(Alliance alliance) {
        if (alliance == Alliance.BLUE) {
            return new Pose(RobotConstants.BLUE_GOAL_X, RobotConstants.BLUE_GOAL_Y);
        }

        return new Pose(RobotConstants.RED_GOAL_X, RobotConstants.RED_GOAL_Y);
    }

    public static Pose getMiddlePose(Alliance alliance) {
        if (alliance == Alliance.BLUE) {
            return new Pose(RobotConstants.BLUE_MIDDLE_X, RobotConstants.BLUE_MIDDLE_Y);
        }

        return new Pose(RobotConstants.RED_MIDDLE_X, RobotConstants.RED_MIDDLE_Y);
    }
}
