package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public enum StartPosition {
    BLUE(new Pose(55, 111, Math.toRadians(180))),
    RED(new Pose(122.46, 119.67, Math.toRadians(40.19))),
    CENTER(new Pose(70.5, 70.5, Math.toRadians(90)));

    public final Pose defaultPose;

    StartPosition(Pose defaultPose) {
        this.defaultPose = defaultPose;
    }
}