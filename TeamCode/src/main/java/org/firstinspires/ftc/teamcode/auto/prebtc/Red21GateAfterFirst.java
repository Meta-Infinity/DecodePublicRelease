package org.firstinspires.ftc.teamcode.auto.prebtc;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoConstants;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.Storage;
import org.firstinspires.ftc.teamcode.util.Teleop;

@Autonomous(name = "RED-21 with 2 spikes", group = "Comp Auto")
public class Red21GateAfterFirst extends OpMode {

    private Robot r;
    private Follower f;
    private Timer fullTimer, shootTimer, rampTimer, waitTimer;
    boolean resetShootTimer = false;
    boolean resetRampTimer = false;
    boolean resetWaitTimer = false;

    private double shootTime = 0.4;
    private double rampTime = 1.8;

    private int currentCycle = 0;

    private double zero = Math.toRadians(0);

    private double lastTime = 0;

    private String pathState;

    //    private final Pose startPose = new Pose(122.46, 119.67, Math.toRadians(40.19));
    private final Pose startPose = AutoConstants.RED_AUTO_START_POSE; //michiana

    private final Pose shootPose = new Pose(84.66, 79.88, zero);
    private final Pose shootFinalPose = new Pose(90, 110, zero);

    private final Pose g1Initial = new Pose(114, 84, zero);
    private final Pose g1cp1 = new Pose(103, 87);
    private final Pose g1cp2 = new Pose(91, 82);

    private final Pose g2Initial = new Pose(117.2, 61.5, zero);
    private final Pose g2cp1 = new Pose(85, 54.5);
    private final Pose g2cp2 = new Pose(102.5, 56.5);
    private final Pose g2cp3 = new Pose(100, 53);

    private final Pose ramp = new Pose(130.6-0.1, 55.5, Math.toRadians(26));
    private final Pose ramp2 = new Pose(130.9-0.1, 55.8, Math.toRadians(26));
    private final Pose ramp3 = new Pose(131.3-0.1, 56.1, Math.toRadians(26));
    private final Pose ramp4 = new Pose(131.7-0.1, 56.3, Math.toRadians(26));

    private final Pose rcp1 = new Pose(105, 56);

    private Path shootPreload;
    private PathChain grabTwo, grabRamp1, returnRamp1, grabRamp2, returnRamp2, grabRamp3, returnRamp3, grabRamp4, returnRamp4, grabOne;

    private double rampEndPower = 0.3;

    public void buildPaths() {
        shootPreload = new Path(new BezierLine(startPose, shootPose));
        shootPreload.setLinearHeadingInterpolation(startPose.getHeading(), zero);

        grabTwo = f.pathBuilder()
                .addPath(new BezierCurve(shootPose, g2cp1, g2cp2, g2Initial))
                .setConstantHeadingInterpolation(zero)
                .addParametricCallback(0.1, setIntakeOn())
                .addPath(new BezierCurve(g2Initial, g2cp3, shootPose))
                .setConstantHeadingInterpolation(zero)
                .build();

        grabRamp1 = f.pathBuilder()
                .addPath((new BezierCurve(shootPose, rcp1, ramp)))
                .setLinearHeadingInterpolation(zero, ramp.getHeading())
                .addParametricCallback(0.625, setMaxPower(rampEndPower))
                .build();

        returnRamp1 = f.pathBuilder()
                .addPath((new BezierCurve(ramp, rcp1, shootPose)))
                .setLinearHeadingInterpolation(0, zero)
                .addParametricCallback(0.0001, setMaxPower(1))
                .addParametricCallback(0.7, setIntakeOn())
                .build();

        grabRamp2 = f.pathBuilder()
                .addPath((new BezierCurve(shootPose, rcp1, ramp2)))
                .setLinearHeadingInterpolation(zero, ramp2.getHeading())
                .addParametricCallback(0.625, setMaxPower(rampEndPower))
                .build();

        returnRamp2 = f.pathBuilder()
                .addPath((new BezierCurve(ramp2, rcp1, shootPose)))
                .setLinearHeadingInterpolation(0, zero)
                .addParametricCallback(0.0001, setMaxPower(1))
                .addParametricCallback(0.7, setIntakeOn())
                .build();

        grabRamp3 = f.pathBuilder()
                .addPath((new BezierCurve(shootPose, rcp1, ramp3)))
                .setLinearHeadingInterpolation(zero, ramp3.getHeading())
                .addParametricCallback(0.625, setMaxPower(rampEndPower))
                .build();

        returnRamp3 = f.pathBuilder()
                .addPath((new BezierCurve(ramp3, rcp1, shootPose)))
                .setLinearHeadingInterpolation(0, zero)
                .addParametricCallback(0.0001, setMaxPower(1))
                .addParametricCallback(0.7, setIntakeOn())
                .build();

        grabRamp4 = f.pathBuilder()
                .addPath((new BezierCurve(shootPose, rcp1, ramp4)))
                .setLinearHeadingInterpolation(zero, ramp4.getHeading())
                .addParametricCallback(0.625, setMaxPower(rampEndPower))
                .build();

        returnRamp4 = f.pathBuilder()
                .addPath((new BezierCurve(ramp4, rcp1, shootPose)))
                .setLinearHeadingInterpolation(0, zero)
                .addParametricCallback(0.0001, setMaxPower(1))
                .addParametricCallback(0.7, setIntakeOn())
                .build();

        grabOne = f.pathBuilder()
                .addPath(new BezierCurve(shootPose, g1cp1, g1Initial))
                .setConstantHeadingInterpolation(zero)
                .addParametricCallback(0.01, setMaxPower(0.65))
                .addParametricCallback(0.1, setIntakeOn())
                .addPath(new BezierCurve(g1Initial, g1cp2, shootFinalPose))
                .setConstantHeadingInterpolation(zero)
                .addParametricCallback(0.6, setMaxPower(1))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case "path":
                if(!f.isBusy()) {
                    r.intake.transferBlock();
                    if (currentCycle == 0) {
                        currentCycle++;
                        f.followPath(shootPreload);
                        set("preloadWait");
                    } else if (currentCycle == 1) {
                        currentCycle++;
                        f.followPath(grabTwo);
                        set("shoot");
                    } else if (currentCycle == 2) {
                        currentCycle++;
                        f.followPath(grabRamp1);
                        set("rampWait");
                    } else if (currentCycle == 3) {
                        currentCycle++;
                        f.followPath(grabRamp2);
                        set("rampWait");
                    } else if (currentCycle == 4) {
                        currentCycle++;
                        f.followPath(grabRamp3);
                        set("rampWait");
                    } else if (currentCycle == 5) {
                        currentCycle++;
                        f.followPath(grabRamp4);
                        set("rampWait");
                    } else if (currentCycle == 6){
                        r.turret.addOffSetDegrees(-6);
                        currentCycle++;
                        f.followPath(grabOne);
                        set("shoot");
                    } else {
                        set("stop");
                    }
                }
                break;

            case "preloadWait":
                if (!f.isBusy()) {
                    if (!resetWaitTimer) {
                        waitTimer.resetTimer();
                        resetWaitTimer = true;
                    }
                    if (waitTimer.getElapsedTimeSeconds() > 0.5) {
                        resetWaitTimer = false;
                        set("shoot");
                    }
                }
                break;

            case "shoot":
                if (!f.isBusy()) {
                    if (!resetShootTimer) {
                        shootTimer.resetTimer();
                        resetShootTimer = true;
                    }
                    r.shooter.isShooting();
                    r.intake.transferShoot();
                    r.intake.run();

                    if (shootTimer.getElapsedTimeSeconds() > shootTime) {
                        r.intake.transferBlock();
                        r.shooter.notShooting();
                        resetShootTimer = false;
                        set("path");
                    }
                }
                break;

            case "rampWait":
                if(!f.isBusy()) {
                    r.intake.run();
                    r.intake.transferRun();
                    if (!resetRampTimer) {
                        rampTimer.resetTimer();
                        resetRampTimer = true;
                    }

                    double currentRampTimeout = (currentCycle == 3) ? 1.0 : rampTime;

                    if (r.intake.getIntakeFull() || r.intake.getArtifactCount() >= 3 || rampTimer.getElapsedTimeSeconds() > currentRampTimeout) {
                        resetRampTimer = false;
                        if(currentCycle == 3){
                            f.followPath((returnRamp1));
                        } else if(currentCycle == 4){
                            f.followPath((returnRamp2));
                        } else if(currentCycle == 5){
                            f.followPath((returnRamp3));
                        } else if(currentCycle == 6){
                            f.followPath((returnRamp4));
                        }
                        set("shoot");
                    }
                }
                break;

            case "stop":
                if(!f.isBusy()) {
                    r.turret.addOffSetDegrees(6);
                    r.turret.stopTrack();
                    r.turret.resetTurret();
                    r.intake.stop();
                    r.shooter.stopTrack();
                    set("end");
                }
                break;
        }
    }

    public Runnable setIntakeOn() { return () -> r.intake.run(); }
    public Runnable setIntakeOff() { return () -> r.intake.stop(); }
    public void set(String pathState) { this.pathState = pathState; }
    public Runnable setMaxPower(double maxPower) { return () -> f.setMaxPower(maxPower); }

    @Override
    public void loop() {
        r.clearCache();
        f.update();
        r.update(f.getPose(), f.getVelocity());
        autonomousPathUpdate();
        Storage.lastPoseRed = f.getPose();
        Storage.lastTurretAngleRed = r.turret.getCurrentAngle();
        telemetry.update();
    }

    @Override
    public void init() {
        fullTimer = new Timer();
        shootTimer = new Timer();
        rampTimer = new Timer();
        waitTimer = new Timer();
        r = new Robot(hardwareMap, Alliance.RED, Teleop.FALSE, startPose);
        r.resetPosAndIMU();
        f = Constants.createFollower(hardwareMap);
        buildPaths();
        f.setStartingPose(startPose);
        Storage.lastPoseRed = startPose;
        Storage.lastTurretAngleRed = r.turret.getCurrentAngle();
        r.intake.transferBlock();
        r.shooter.setHood(0.5);
        r.turret.setAngle(0);
    }

    @Override
    public void init_loop() {
        r.clearCache();
        r.turret.setAngle(0);
        r.turret.update(0, 0, 0);
        telemetry.addLine("Red Close 21");
        telemetry.update();
    }

    @Override
    public void start() {
        fullTimer.resetTimer();
        set("path");
        r.intake.transferBlock();
        r.intake.run();
        r.turret.startTrack();
        r.shooter.startTrack();
    }

    @Override
    public void stop() { r.turret.stopTrack(); }
}