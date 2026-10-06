package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public class Storage {
    // We initialize to null to signify "uninitialized/fresh boot"
    public static Pose lastPoseRed = null;
    public static Pose lastPoseBlue = null;
    
    // We use Double.NaN to signify the turret state hasn't been saved yet
    public static double lastTurretAngleRed = Double.NaN;
    public static double lastTurretAngleBlue = Double.NaN;
}
