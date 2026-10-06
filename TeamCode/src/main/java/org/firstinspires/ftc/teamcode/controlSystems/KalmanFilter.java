package org.firstinspires.ftc.teamcode.controlSystems;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.RobotConstants;


public class KalmanFilter {

    private double x;
    private double y;

    private double px;
    private double py;

    Pose lastPose;


    public KalmanFilter(Pose startPose, double q, double r) {

        lastPose = startPose;

        x = startPose.getX();
        y = startPose.getY();


        this.px = 1;
        this.py = 1;


       RobotConstants.KALMAN_Q = q;
       RobotConstants.KALMAN_R = r;

    }

    public void predict(Pose odomPose) {
        x += odomPose.getX() - lastPose.getX();
        y += odomPose.getY() - lastPose.getY();


        px += RobotConstants.KALMAN_Q;
        py += RobotConstants.KALMAN_Q;



        lastPose = odomPose;
    }


    public void correct(Pose llPose, double distance) {

        double scale = distance / 70;
        double rAdjusted = RobotConstants.KALMAN_R * (1 + scale * scale);

        rAdjusted = Math.min(rAdjusted, 10.0);


        // kalmain gain
        double kx = px / (px + rAdjusted);
        double ky = py / (py + rAdjusted);


        double dx = llPose.getX() - x;
        double dy = llPose.getY() - y;
        if (Math.hypot(dx, dy) > 90) return;


        x += kx * (llPose.getX() - x);
        y += ky * (llPose.getY() - y);


        // update uncertainty
        px = (1 - kx) * px;
        py = (1 - ky) * py;

    }


    public Pose getFusionPose(double heading) {
        return new Pose(x, y, heading);
    }




    public double getX() { return x; }
    public double getY() { return y; }

}
