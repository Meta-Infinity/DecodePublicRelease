package org.firstinspires.ftc.teamcode.auto;

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
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.Storage;
import org.firstinspires.ftc.teamcode.util.Teleop;

@Autonomous(name = "RED-24 close first 2 spikes", group = "COMP Auto")
public class RED24CLOSE extends OpMode {

    private Robot r;
    private Follower f;
    private Timer fullTimer, shootTimer, rampTimer, waitTimer;
    private boolean resetShootTimer = false;
    private boolean resetRampTimer = false;
    private boolean resetWaitTimer = false;

    private final double shootTime = 0.3;
    private final double rampTime = 1.6;
    // Its positive equivalent is just outside the soft limit, so remapping must select the negative loop.
    private final double otherTurretLoopAngle =
            RobotConstants.getTurretSoftMaxDeg() + 1 - 360;

    private int currentCycle = 0;
    private final double zero = Math.toRadians(0);
    private String pathState;

    private final Pose startPose = AutoConstants.RED_AUTO_START_POSE;
    private final Pose shootPose = new Pose(89.1, 85.9, zero);

    private final Pose spike1 = new Pose(114, 82.67, zero);
    private final Pose shootPoseAfterSpike1 = new Pose(90.42, 85, zero);
    private final Pose spike2GrabControl = new Pose(93, 48, zero);
    private final Pose spike2 = new Pose(123.5, 60, zero);
    private final Pose returnGate = new Pose(92.03, 81.91, zero);
    private final Pose toGate = new Pose(103.8, 71.7, zero);

        private final Pose gateIntake1 = new Pose(130.8, 56.5, Math.toRadians(26));
        private final Pose gateIntake2 = new Pose(130.8, 56.6, Math.toRadians(26));
        private final Pose gateIntake3 = new Pose(131.0, 57.0, Math.toRadians(26));
        private final Pose gateIntake4 = new Pose(131.0, 57.1, Math.toRadians(26));
        private final Pose gateIntake5 = new Pose(131.2, 57.1, Math.toRadians(26));
        private final Pose gateIntake6 = new Pose(131.3, 57.3, Math.toRadians(26));
        private final Pose gateRetryPoint = new Pose(120, 50, Math.toRadians(26));
        private final Pose park = new Pose(102.73, 78.76, zero);

    private Path shootPreload;
    private PathChain Spike1, grabTwo, parkPath;

    private Pose activeGateBase;
    private Pose activeGateTarget;
    private double gateYOffset = 0;
    private int gateRetryCount = 0;
    private int detectedArtifactCount = 0;

    public void buildPaths() {
        shootPreload = new Path(new BezierLine(startPose, shootPose));
        shootPreload.setTangentHeadingInterpolation();
        shootPreload.reverseHeadingInterpolation();

        Spike1 = f.pathBuilder()
                .addPath(new BezierLine(shootPose, spike1))
                .setConstantHeadingInterpolation(zero)
                .addParametricCallback(0.1, setIntakeOn())
                .addPath(new BezierLine(spike1, shootPoseAfterSpike1))
                .setConstantHeadingInterpolation(zero)
                .build();

        grabTwo = f.pathBuilder()
                .addPath(new BezierCurve(shootPose, spike2GrabControl, spike2))
                .setConstantHeadingInterpolation(zero)
                .addParametricCallback(0.1, setIntakeOn())
                .build();

        parkPath = f.pathBuilder()
                .addPath(new BezierLine(returnGate, park))
                .setTangentHeadingInterpolation()
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case "path":
                if (!f.isBusy()) {
                    r.intake.transferBlock();
                    if (currentCycle == 0) {
                        currentCycle++;
                        f.followPath(shootPreload);
                        set("preloadWait");
                    } else if (currentCycle == 1) {
                        currentCycle++;
                        r.turret.resetOffsetDegrees();
                        r.turret.addOffSetDegrees(-5);
                        f.followPath(Spike1);
                        set("shoot");
                    }
                        else if (currentCycle == 2) {
                        currentCycle++;
                        f.followPath(grabTwo);
                        set("grabTwoReturn");
                    } else if (currentCycle == 3) {
                        currentCycle++;
                        startGateIntake(gateIntake1, false);
                    } else if (currentCycle == 4) {
                        currentCycle++;
                        startGateIntake(gateIntake2, false);
                    } else if (currentCycle == 5) {
                        currentCycle++;
                        startGateIntake(gateIntake3, false);
                    } else if (currentCycle == 6) {
                        currentCycle++;
                        startGateIntake(gateIntake4, false);
                    } else if (currentCycle == 7) {
                        currentCycle++;
                        startGateIntake(gateIntake5, false);
                    } else if (currentCycle == 8) {
                        currentCycle++;
                        f.followPath(parkPath);
                        set("park");
                    }
//                        else if (currentCycle == 9) {
//                        currentCycle++;
//                        f.followPath(parkPath);
//                        set("park");
//                    }
                        else {
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
                    if (waitTimer.getElapsedTimeSeconds() > 0.2) {
                        resetWaitTimer = false;
                        set("shoot");
                    }
                }
                break;

            case "shoot":
                if (!f.isBusy() && r.intake.prepareShoot()) {
                    if (!resetShootTimer) {
                        shootTimer.resetTimer();
                        resetShootTimer = true;
                    }
                    r.shooter.isShooting();
                    r.intake.transferShoot();
                    r.intake.run();
                    r.intake.run();

                    if (shootTimer.getElapsedTimeSeconds() > shootTime) {
                        r.intake.transferBlock();
                        r.shooter.notShooting();
                        r.intake.shootComplete();
                        resetShootTimer = false;
                        if (currentCycle == 1) {
                            r.turret.startTrack();
                        }
                        else if (currentCycle == 2) {
                            r.turret.resetOffsetDegrees();
                            r.turret.stopTrack();
                            r.turret.setAngle(otherTurretLoopAngle);
                        }
                        set("path");
                    }
                }
                break;

            case "grabTwoReturn":
                if (!f.isBusy()) {
                    r.turret.startTrack();
                    returnToShoot(spike2);
                }
                break;

            case "rampWait":
                if (!f.isBusy()) {
                    r.intake.run();
                    r.intake.transferRun();
                    if (!resetRampTimer) {
                        rampTimer.resetTimer();
                        resetRampTimer = true;
                    }

                    if (r.intake.getIntakeFull()
                            || detectedArtifactCount >= 3
                            || rampTimer.getElapsedTimeSeconds() > rampTime) {
                        resetRampTimer = false;
                        if (currentCycle >= 3 && currentCycle <= 8) {
                            if (gateIntakeIsEmpty()) {
                                gateYOffset -= .4;
                                gateRetryCount++;
                                startGateIntake(activeGateBase, true);
                            } else {
                                returnToShoot(activeGateTarget);
                            }
                        } else {
                            set("shoot");
                        }
                    }
                }
                break;

            case "park":
                if (!f.isBusy()) {
                    set("stop");
                }
                break;

            case "stop":
                if (!f.isBusy()) {
                    r.turret.stopTrack();
                    r.turret.resetTurret();
                    r.intake.stop();
                    r.shooter.stopTrack();
                    set("end");
                }
                break;
        }
    }

    private void startGateIntake(Pose baseTarget, boolean retry) {
        activeGateBase = baseTarget;
        activeGateTarget = new Pose(
                baseTarget.getX(),
                baseTarget.getY() + gateYOffset,
                baseTarget.getHeading());

        PathChain intakePath;
        if (retry) {
            intakePath = f.pathBuilder()
                    .addPath(new BezierLine(f.getPose(), gateRetryPoint))
                    .setConstantHeadingInterpolation(activeGateTarget.getHeading())
                    .addPath(new BezierLine(gateRetryPoint, activeGateTarget))
                    .setConstantHeadingInterpolation(activeGateTarget.getHeading())
                    .build();
        } else {
            intakePath = f.pathBuilder()
                    .addPath(new BezierLine(returnGate, toGate))
                    .setTangentHeadingInterpolation()
                    .addPath(new BezierLine(toGate, activeGateTarget))
                    .setConstantHeadingInterpolation(activeGateTarget.getHeading())
                    .build();
        }

        f.followPath(intakePath);
        set("rampWait");
    }

    private void returnToShoot(Pose intakeTarget) {
        PathChain returnPath = f.pathBuilder()
                .addPath(new BezierLine(intakeTarget, returnGate))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.0001, setMaxPower(1))
                .addParametricCallback(0.7, setIntakeOn())
                .build();
        f.followPath(returnPath);
        set("shoot");
    }

    private boolean gateIntakeIsEmpty() {
        return detectedArtifactCount == 0
                && !r.intake.dbg_beam1
                && !r.intake.dbg_beam2
                && !r.intake.dbg_beam3;
    }

    public Runnable setIntakeOn() {
        return () -> r.intake.run();
    }

    public void set(String pathState) {
        this.pathState = pathState;
    }

    public Runnable setMaxPower(double maxPower) {
        return () -> f.setMaxPower(maxPower);
    }

    @Override
    public void loop() {
        r.clearCache();
        f.update();
        r.update(f.getPose(), f.getVelocity());
        // Intake.update() already read and filtered the break beams during r.update().
        // Cache that result once for both autonomous decisions and telemetry.
        detectedArtifactCount = r.intake.getArtifactCount();
        autonomousPathUpdate();
        Storage.lastPoseRed = f.getPose();
        Storage.lastTurretAngleRed = r.turret.getCurrentAngle();
        telemetry.addData("State", pathState);
        telemetry.addData("Balls detected", detectedArtifactCount);
        telemetry.addData("Gate retry count", gateRetryCount);
        telemetry.addData("Gate position Y increment", "%.1f in", gateYOffset);
        if (activeGateTarget != null) {
            telemetry.addData("Gate target", "(%.1f, %.1f)",
                    activeGateTarget.getX(), activeGateTarget.getY());
        }
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
        telemetry.addLine("RED-24 RIGHT SKIP1 PRE-ROTATE");
        telemetry.update();
    }

    @Override
    public void start() {
        fullTimer.resetTimer();
        set("path");
        r.intake.transferBlock();
        r.intake.run();
        r.turret.setAngle(0);
        r.shooter.startTrack();
    }

    @Override
    public void stop() {
        r.turret.stopTrack();
    }
}
