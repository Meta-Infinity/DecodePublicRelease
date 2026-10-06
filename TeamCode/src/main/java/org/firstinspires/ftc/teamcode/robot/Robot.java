package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainHardware;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.controlSystems.KalmanFilter;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.controlSystems.LookUpTable;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.Teleop;

import java.util.List;


public class Robot {
    public DrivetrainHardware drivetrain;
    public Intake intake;
     public Limelight limelight;
    public Shooter shooter;
    public Turret turret;
    public LookUpTable storage;
     public KalmanFilter kf;
    public Lights lights;
    private final GoBildaPinpointDriver pinpoint;
    List<LynxModule> allHubs;

    private Pose fusionPose;
    private Pose odomPose;
    private Pose trackPose;
    private Pose goalPose;
    private boolean targetingMiddle;

    private boolean sotm = false;
    private boolean kfStatus = false;

    private final Teleop isTeleop;
    private final Alliance alliance;

    private double xOffset = 0;
    private double yOffset = 0;

    public Robot(HardwareMap hardwareMap, Alliance alliance, Teleop teleop, Pose startPose){
        drivetrain = new DrivetrainHardware(hardwareMap);
         kf = new KalmanFilter(startPose, 0.003, 8);

        intake = new Intake(hardwareMap, teleop);
         limelight = new Limelight(hardwareMap, alliance);
        storage = new LookUpTable();


        shooter = new Shooter(hardwareMap, storage);
        turret = new Turret(hardwareMap, 0, RobotConfig.getEncoderWrapOffsetDeg(alliance));

        lights = new Lights(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        allHubs = hardwareMap.getAll(LynxModule.class);

        this.alliance = alliance;
        fusionPose = startPose;
        odomPose = startPose;
        targetingMiddle = false;
        goalPose = FieldConstants.getGoalPose(alliance);
        trackPose = goalPose;

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


         limelight.initalize();
        isTeleop = teleop;
    }


    public Pose getFusionPose(){
        return fusionPose;
    }
    public Pose getLLPose(){
        if(limelight.canSeeTarget()) {
            return limelight.getPose(Math.toDegrees(odomPose.getHeading()));
        } else {
            return new Pose(0,0,0);
        }
    }
    public Pose getLLRawPose(){
        if(limelight.canSeeTarget()) {
            Pose raw = limelight.getRawPose();
            return raw != null ? raw : new Pose(0,0,0);
        } else {
            return new Pose(0,0,0);
        }
    }

    public double getLLDistance() {
        if (limelight.canSeeTarget()) {
            return limelight.getDistanceInches();
        } else {
            return -1;
        }
    }

    public void sotmOn(){
        sotm = true;
    }
    public void sotmOff(){
        sotm = false;
    }

    public void kfOn(){
        kfStatus = true;
    }
    public void kfOff(){
        kfStatus = false;
    }

    public Pose getTrackPose(){
        return trackPose;
    }
    public boolean getSOTM(){
        return sotm;
    }

    public void clearCache(){
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public double round(double x) {
        return Math.round(x * 100.0) / 100.0;
    }

    public Pose getGoalPose() {
        return goalPose;
    }

    public void setGoalPose(Pose goalPose) {
        this.goalPose = goalPose;
        trackPose = goalPose;
    }

    public String getTargetName() {
        return targetingMiddle ? alliance + " MIDDLE" : alliance + " GOAL";
    }

    public void switchGoal() {
        targetingMiddle = !targetingMiddle;
        setGoalPose(targetingMiddle
                ? FieldConstants.getMiddlePose(alliance)
                : FieldConstants.getGoalPose(alliance));
    }
    
    public void resetPosAndIMU() {
        pinpoint.resetPosAndIMU();
    }


    public void update(Pose pose, Vector velocity){

        odomPose = pose;

        if(kfStatus){
            limelight.update(Math.toDegrees(odomPose.getHeading()));

            kf.predict(pose);
            if(limelight.canSeeTarget()){
                Pose llPose = limelight.getPose(Math.toDegrees(odomPose.getHeading()));

                if (llPose != null) {

                    Pose poseForCorrection = new Pose(llPose.getX(), llPose.getY(), llPose.getHeading());
                    kf.correct(poseForCorrection, limelight.getDistanceInches());
                }
            }
            fusionPose = kf.getFusionPose(odomPose.getHeading());
            xOffset = fusionPose.getX() - odomPose.getX();
            yOffset = fusionPose.getY() - odomPose.getY();
        } else{
            fusionPose = new Pose(odomPose.getX() + xOffset, odomPose.getY() + yOffset, odomPose.getHeading());
        }


        // Account for the turret's physical offset from the robot center
        double cos = Math.cos(fusionPose.getHeading());
        double sin = Math.sin(fusionPose.getHeading());

        double turretFieldX = fusionPose.getX() + (RobotConstants.TURRET_X_OFFSET * cos - RobotConstants.TURRET_Y_OFFSET * sin);
        double turretFieldY = fusionPose.getY() + (RobotConstants.TURRET_X_OFFSET * sin + RobotConstants.TURRET_Y_OFFSET * cos);

        double staticDx = goalPose.getX() - turretFieldX;
        double staticDy = goalPose.getY() - turretFieldY;
        double staticDistance = Math.sqrt(staticDx * staticDx + staticDy * staticDy);

        if(sotm && (staticDistance > 30.0 || staticDistance < 110)){ // yo wtf are we doing bro ts is true every time :sob:
            double flightTime = storage.getTime(staticDistance);
            trackPose = new Pose(goalPose.getX() - velocity.getXComponent()*flightTime, goalPose.getY() - velocity.getYComponent()*flightTime);
        } else{
            trackPose = goalPose;
        }

        double dx = trackPose.getX() - turretFieldX;
        double dy = trackPose.getY() - turretFieldY;

        double distanceToGoal = Math.sqrt(dx * dx + dy * dy);

        double voltage = 12.0;
        if (!allHubs.isEmpty()) {
            voltage = allHubs.get(0).getInputVoltage(VoltageUnit.VOLTS);
        }

        shooter.update(distanceToGoal, voltage);
        turret.update(dx, dy, pose.getHeading(), pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS), voltage);

        intake.setDynamicTransferPower(storage.getTransferPower(distanceToGoal));
        intake.update();

    }


}
