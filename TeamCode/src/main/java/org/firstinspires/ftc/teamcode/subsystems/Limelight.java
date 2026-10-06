package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.Alliance;

public class Limelight {
    private Limelight3A limelight;
    private int pipeline;
    private LLResult result;
    private double lastDistance;
    private boolean canSeeTarget;

    private static double LL_HEIGHT_INCHES = 8.39173228;
    private static double GOAL_HEIGHT_INCHES = 29.5;
    private static double LL_ANGLE_DEGREES = 17.102729;
    private static double METER_TO_INCH = 39.37007;

    public Limelight(HardwareMap hardwareMap, Alliance alliance){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(60);
        switch (alliance) {
            case RED:
                pipeline = 0;
                break;
            case BLUE:
                pipeline = 1;
                break;
            default:
                pipeline = 0;
        }
    }

    public void initalize(){
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }


    public double getDistanceInches(){
        if((result != null) && (result.isValid())){
            double tagVerticalAngleDegrees = result.getTy();
            double totalAngleRadians = (Math.PI / 180) * (LL_ANGLE_DEGREES + tagVerticalAngleDegrees);
            lastDistance = (GOAL_HEIGHT_INCHES - LL_HEIGHT_INCHES) / Math.tan(totalAngleRadians);
        }
        return lastDistance;
    }

    //works
    public Pose getPose(double heading) {

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double rawY = -(botpose.getPosition().x) * METER_TO_INCH + 70.5;
                double rawX = (botpose.getPosition().y) * METER_TO_INCH + 70.5;

                double headingRad = Math.toRadians(heading);


                return new Pose(rawX, rawY);
            }
        }
        return null;
    }


    public Pose getRawPose() {
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
               double rawX = botpose.getPosition().x;
               double rawY = botpose.getPosition().y;
               double rawH = botpose.getOrientation().getYaw();

                return new Pose(rawX, rawY, rawH);
            }
        }
        return null;
    }

    public boolean canSeeTarget(){
        return canSeeTarget;
    }

    public void update(double heading){
        result = limelight.getLatestResult();
        canSeeTarget = (result != null && result.isValid());
    }

}
