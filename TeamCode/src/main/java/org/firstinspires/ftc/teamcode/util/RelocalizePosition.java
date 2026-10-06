package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public enum RelocalizePosition {
    BLUE_CORNER(new Pose(140.5-9, 9, Math.toRadians(270))),
    RED_CORNER(new Pose(9, 9, Math.toRadians(270))),
    //    RED_GATE(new Pose(126.93, 79.62, 0)),
//    RED_GATE(new Pose(126.93, 79.62, 0)),
//    RED_GATE(new Pose(125.5, 77.7, 0)), //michiana value
//    BLUE_GATE(new Pose(16.00, 79.21, 3.14159)), //michiana value

    RED_GATE(new Pose(124.56, 77.25, Math.toRadians(-0.57))), //michiana value
    BLUE_GATE(new Pose(16.52, 79.11, Math.toRadians(179.33))), //michiana value


//    BLUE_GATE(new Pose(13.76, 78.22, 3.14)), //practice value
//    BLUE_GATE(new Pose(15.36, 79.09, 3.13)), //worlds value
//        BLUE_GATE(new Pose(17.01, 78.66, 3.12)), //home value

    CENTER(new Pose(70.5, 70.5, Math.toRadians(90)));

    public final Pose defaultPose;

    RelocalizePosition(Pose defaultPose) {
        this.defaultPose = defaultPose;
    }
}
