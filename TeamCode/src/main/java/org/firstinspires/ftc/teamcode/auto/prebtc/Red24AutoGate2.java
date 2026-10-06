package org.firstinspires.ftc.teamcode.auto.prebtc;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoConstants;
import org.firstinspires.ftc.teamcode.util.Storage;
import org.firstinspires.ftc.teamcode.util.Teleop;

@Config
@Autonomous(name = "RED24AUTOGATE2", group = "Practice Auto")
public class Red24AutoGate2 extends OpMode {

    public static double GATE_CONTACT_VELOCITY_INCHES_PER_SECOND = 1.0;
    public static double GATE_CONTACT_ARM_VELOCITY_INCHES_PER_SECOND = 2.0;
    public static double GATE_STALL_CONFIRM_SECONDS = 0.20;
    public static double GATE_MIN_PROBE_SECONDS = 0.10;
    public static double GATE_ATTEMPT_TIMEOUT_SECONDS = 3.0;
    public static double GATE_MIN_DISTANCE_REMAINING = 1.0;
    public static double GATE_RETRY_Y_STEP = 0.30;
    public static int GATE_MAX_RETRIES = 3;
    public static double GATE_INTAKE_SECONDS = 1.6;
    public static int GATE_CYCLES = 5;

    private static final double ZERO = 0;
    private static final double GATE_HOLD_HEADING = Math.toRadians(26);

    private final Pose startPose = AutoConstants.RED_AUTO_START_POSE;
    private final Pose shootPose = new Pose(89.1, 85.9, ZERO);
    private final Pose spike1 = new Pose(119, 82.67, ZERO);
    private final Pose shootPoseAfterSpike1 = new Pose(90.42, 85, ZERO);
    private final Pose spike2GrabControl = new Pose(93, 48, ZERO);
    private final Pose spike2 = new Pose(122, 59.1, ZERO);
    private final Pose returnGate = new Pose(92.03, 81.91, ZERO);
    private final Pose toGate = new Pose(103.8, 71.7, ZERO);
    private final Pose gateIntake = new Pose(130.8, 55.6, GATE_HOLD_HEADING);
    private final Pose gateRetryBackoff = new Pose(126.5, 58.2, GATE_HOLD_HEADING);
    private final Pose park = new Pose(102.73, 78.76, ZERO);

    private Robot robot;
    private Follower follower;
    private Timer shootTimer;
    private Timer intakeTimer;
    private Timer waitTimer;
    private Timer gateProbeTimer;
    private Timer gateStallTimer;

    private Path shootPreload;
    private PathChain spikeOne;
    private PathChain grabTwo;
    private PathChain gateApproach;
    private PathChain gateRetry;
    private PathChain returnFromGate;
    private PathChain parkPath;

    private String pathState;
    private int initialStep;
    private int gateCyclesCompleted;
    private boolean resetShootTimer;
    private boolean resetIntakeTimer;
    private boolean resetWaitTimer;
    private boolean timingGateStall;
    private boolean gateProbeArmed;
    private boolean gateDetectionActive;
    private boolean gateContactDetected;
    private boolean turretTracking;
    private int gateRetryCount;
    private Pose gateAttemptTarget;
    private double gateAttemptHeading;
    private double gateVelocity;

    public void buildPaths() {
        shootPreload = new Path(new BezierLine(startPose, shootPose));
        shootPreload.setTangentHeadingInterpolation();
        shootPreload.reverseHeadingInterpolation();

        spikeOne = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, spike1))
                .setConstantHeadingInterpolation(ZERO)
                .addParametricCallback(0.1, this::startIntake)
                .addPath(new BezierLine(spike1, shootPoseAfterSpike1))
                .setConstantHeadingInterpolation(ZERO)
                .build();

        grabTwo = follower.pathBuilder()
                .addPath(new BezierCurve(shootPoseAfterSpike1, spike2GrabControl, spike2))
                .setConstantHeadingInterpolation(ZERO)
                .addParametricCallback(0.1, this::startIntake)
                .addPath(new BezierLine(spike2, returnGate))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        gateApproach = follower.pathBuilder()
                .addPath(new BezierLine(returnGate, toGate))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(toGate, gateIntake))
                .setConstantHeadingInterpolation(GATE_HOLD_HEADING)
                .build();

        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(returnGate, park))
                .setTangentHeadingInterpolation()
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case "path":
                startNextPath();
                break;

            case "preloadWait":
                if (!follower.isBusy()) {
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
                shootArtifacts();
                break;

            case "spikeTwoIntake":
                runIntakeAtGate(1.0);
                break;

            case "gateApproach":
                detectGateContact();
                break;

            case "gateHold":
                runIntakeAtGate(GATE_INTAKE_SECONDS);
                break;

            case "park":
                if (!follower.isBusy()) {
                    set("stop");
                }
                break;

            case "stop":
                robot.turret.stopTrack();
                robot.turret.resetTurret();
                robot.intake.stop();
                robot.shooter.stopTrack();
                set("end");
                break;
        }
    }

    private void startNextPath() {
        if (follower.isBusy()) {
            return;
        }

        robot.intake.transferBlock();
        if (initialStep == 0) {
            initialStep++;
            follower.followPath(shootPreload);
            set("preloadWait");
        } else if (initialStep == 1) {
            initialStep++;
            follower.followPath(spikeOne);
            set("shoot");
        } else if (initialStep == 2) {
            initialStep++;
            follower.followPath(grabTwo);
            set("spikeTwoIntake");
        } else if (gateCyclesCompleted < GATE_CYCLES) {
            gateCyclesCompleted++;
            startBaseGateAttempt();
        } else {
            follower.followPath(parkPath);
            set("park");
        }
    }

    private void shootArtifacts() {
        if (follower.isBusy()) {
            return;
        }

        if (!robot.intake.prepareShoot()) {
            return;
        }

        if (!resetShootTimer) {
            shootTimer.resetTimer();
            resetShootTimer = true;
        }

        robot.shooter.isShooting();
        robot.intake.transferShoot();
        robot.intake.run();

        if (shootTimer.getElapsedTimeSeconds() > 0.3) {
            robot.intake.transferBlock();
            robot.shooter.notShooting();
            robot.intake.shootComplete();
            resetShootTimer = false;
            if (!turretTracking) {
                robot.turret.startTrack();
                turretTracking = true;
            }
            set("path");
        }
    }

    private void startBaseGateAttempt() {
        gateRetryCount = 0;
        gateAttemptTarget = gateIntake;
        gateAttemptHeading = Math.atan2(
                gateIntake.getY() - toGate.getY(),
                gateIntake.getX() - toGate.getX());
        resetGateDetection();
        follower.setMaxPower(1);
        follower.followPath(gateApproach);
        set("gateApproach");
    }

    private void resetGateDetection() {
        gateDetectionActive = false;
        gateProbeArmed = false;
        timingGateStall = false;
        gateContactDetected = false;
        gateVelocity = 0;
    }

    private void detectGateContact() {
        if (follower.getChainIndex() < 1) {
            if (!follower.isBusy()) {
                startGateRetry();
            }
            return;
        }

        if (!gateDetectionActive) {
            gateDetectionActive = true;
            gateProbeTimer.resetTimer();
            gateStallTimer.resetTimer();
        }

        gateVelocity = Math.abs(follower.getVelocity().dot(new Vector(1, gateAttemptHeading)));

        if (gateVelocity >= GATE_CONTACT_ARM_VELOCITY_INCHES_PER_SECOND) {
            gateProbeArmed = true;
        }

        boolean hasRoomToProbe = follower.getDistanceRemaining() > GATE_MIN_DISTANCE_REMAINING;
        boolean isMovingSlowly = gateVelocity < GATE_CONTACT_VELOCITY_INCHES_PER_SECOND;
        boolean canDetectContact = follower.isBusy()
                && gateProbeArmed
                && gateProbeTimer.getElapsedTimeSeconds() > GATE_MIN_PROBE_SECONDS
                && hasRoomToProbe && isMovingSlowly;

        if (!follower.isBusy() || gateProbeTimer.getElapsedTimeSeconds() > GATE_ATTEMPT_TIMEOUT_SECONDS) {
            startGateRetry();
            return;
        }

        if (!canDetectContact) {
            timingGateStall = false;
            return;
        }

        if (!timingGateStall) {
            gateStallTimer.resetTimer();
            timingGateStall = true;
        }

        if (gateStallTimer.getElapsedTimeSeconds() > GATE_STALL_CONFIRM_SECONDS) {
            gateContactDetected = true;
            holdGatePosition(true);
        }
    }

    private void startGateRetry() {
        if (gateRetryCount >= GATE_MAX_RETRIES) {
            holdGatePosition(false);
            return;
        }

        gateRetryCount++;
        double retryYOffset = gateRetryCount * GATE_RETRY_Y_STEP;
        Pose retryBackoff = new Pose(
                gateRetryBackoff.getX(),
                gateRetryBackoff.getY() + retryYOffset,
                GATE_HOLD_HEADING);
        gateAttemptTarget = new Pose(
                gateIntake.getX(),
                gateIntake.getY() + retryYOffset,
                GATE_HOLD_HEADING);
        gateAttemptHeading = Math.atan2(
                gateAttemptTarget.getY() - retryBackoff.getY(),
                gateAttemptTarget.getX() - retryBackoff.getX());
        gateRetry = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), retryBackoff))
                .setConstantHeadingInterpolation(GATE_HOLD_HEADING)
                .addPath(new BezierLine(retryBackoff, gateAttemptTarget))
                .setConstantHeadingInterpolation(GATE_HOLD_HEADING)
                .build();
        resetGateDetection();
        follower.setMaxPower(1);
        follower.followPath(gateRetry);
    }

    private void holdGatePosition(boolean contactDetected) {
        follower.setMaxPower(1);
        follower.holdPoint(gateAttemptTarget, true);
        gateContactDetected = contactDetected;
        resetIntakeTimer = false;
        set("gateHold");
    }

    private void runIntakeAtGate(double intakeSeconds) {
        robot.intake.run();
        robot.intake.transferRun();
        if (!resetIntakeTimer) {
            intakeTimer.resetTimer();
            resetIntakeTimer = true;
        }

        if (robot.intake.getIntakeFull() || robot.intake.getArtifactCount() >= 3
                || intakeTimer.getElapsedTimeSeconds() > intakeSeconds) {
            resetIntakeTimer = false;
            if (pathState.equals("gateHold")) {
                returnFromGate();
            } else {
                set("shoot");
            }
        }
    }

    private void returnFromGate() {
        Pose currentPose = follower.getPose();
        follower.setMaxPower(1);
        returnFromGate = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, returnGate))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.7, this::startIntake)
                .build();
        follower.followPath(returnFromGate);
        set("shoot");
    }

    private void startIntake() {
        robot.intake.run();
    }

    private void set(String newPathState) {
        pathState = newPathState;
    }

    @Override
    public void loop() {
        robot.clearCache();
        follower.update();
        robot.update(follower.getPose(), follower.getVelocity());
        autonomousPathUpdate();
        Storage.lastPoseRed = follower.getPose();
        Storage.lastTurretAngleRed = robot.turret.getCurrentAngle();
        telemetry.addData("State", pathState);
        telemetry.addData("Gate cycle", "%d/%d", gateCyclesCompleted, GATE_CYCLES);
        telemetry.addData("Gate velocity", "%.2f in/s", gateVelocity);
        telemetry.addData("Gate probe armed", gateProbeArmed);
        telemetry.addData("Gate retry", "%d/%d", gateRetryCount, GATE_MAX_RETRIES);
        telemetry.addData("Gate contact", gateContactDetected);
        telemetry.update();
    }

    @Override
    public void init() {
        shootTimer = new Timer();
        intakeTimer = new Timer();
        waitTimer = new Timer();
        gateProbeTimer = new Timer();
        gateStallTimer = new Timer();
        robot = new Robot(hardwareMap, Alliance.RED, Teleop.FALSE, startPose);
        robot.resetPosAndIMU();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        Storage.lastPoseRed = startPose;
        Storage.lastTurretAngleRed = robot.turret.getCurrentAngle();
        robot.intake.transferBlock();
        robot.shooter.setHood(0.5);
        robot.turret.setAngle(0);
    }

    @Override
    public void init_loop() {
        robot.clearCache();
        robot.turret.setAngle(0);
        robot.turret.update(0, 0, 0);
        telemetry.addLine("RED24AUTOGATE2");
        telemetry.addData("Contact velocity", "< %.2f in/s", GATE_CONTACT_VELOCITY_INCHES_PER_SECOND);
        telemetry.update();
    }

    @Override
    public void start() {
        set("path");
        robot.intake.transferBlock();
        robot.intake.run();
        robot.turret.setAngle(0);
        robot.shooter.startTrack();
    }

    @Override
    public void stop() {
        robot.turret.stopTrack();
    }
}
