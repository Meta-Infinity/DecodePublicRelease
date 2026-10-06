package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Storage;
import org.firstinspires.ftc.teamcode.util.Teleop;

@Autonomous(name = "RED-18 far", group = "Comp Auto")
public class RED18FAR extends OpMode {

    private Robot r;
    private Follower f;
    private Timer fullTimer, shootTimer, cornerTimer, spinUpTimer;
    boolean resetShootTimer = false;
    boolean resetCornerTimer = false;

    private double spinUpTime = 1.5;
    private double shootTime = 1.1;
    private double waitShootTime = 0.01;
    private double cornerTime = 0.4;
    private int cycleCount = 0;
    private int maxCycle = 6;

    private double zero = Math.toRadians(0);

    private String pathState;
    private final Pose startPose = new Pose(84.0, 6.88, zero);

    private final Pose corner = new Pose(133.73,  9.21, zero);
    private final Pose corner2 = new Pose(127, 10, zero);
    private final Pose cornercp1 = new Pose(107, 6.5);

    private final Pose shootPose = new Pose(89, 10, zero);

    private final Pose g3Initial = new Pose(120, 35, zero);
    private final Pose g3cp1 = new Pose(85, 38);
    private final Pose g3Rcp1 = new Pose(86, 36);

    private final Pose cornerInitial = new Pose(126, 8, zero);
    private final Pose cornerIcp1 = new Pose(90, 8);

    private final Pose cornerTurnStrafe = new Pose(126, 20, Math.toRadians(55));
    private final Pose cornerFinalStrafe = new Pose(126, 40, Math.toRadians(55));

    private final Pose parkPose = new Pose(105, 10, zero);

    private PathChain grabThree, grabCorner, grabCornerReturn, cycleCornerInitial, cycleCornerEnding, park, grabCorner2, grabCornerReturn2;

    private double cornerEndPower = 0.6;

    public void buildPaths() {

        grabCorner = f.pathBuilder()
                .addPath(new BezierLine(f.getPose(), corner))
                .setConstantHeadingInterpolation(zero)
                .addParametricCallback(0.001, setMaxPower(1))
                .addParametricCallback(0.8, setMaxPower(cornerEndPower))
                .build();

        grabCornerReturn = f.pathBuilder()
                .addPath(new BezierCurve(corner, cornercp1, shootPose))
                .setConstantHeadingInterpolation(zero)
                .addParametricCallback(0.3, setIntakeOff())
                .build();

        grabCorner2 = f.pathBuilder()
                .addPath(new BezierLine(f.getPose(), corner2))
                .setConstantHeadingInterpolation(zero)
                .addParametricCallback(0.001, setMaxPower(1))
                .addParametricCallback(0.002, setIntakeOn())
                .addParametricCallback(0.8, setMaxPower(cornerEndPower))
                .build();

        grabCornerReturn2 = f.pathBuilder()
                .addPath(new BezierCurve(corner2, cornercp1, shootPose))
                .setConstantHeadingInterpolation(zero)
                //.addParametricCallback(0.2, setIntakeReverse())
                .addParametricCallback(0.3, setIntakeOff())
                .build();

        grabThree = f.pathBuilder()
                .addPath(new BezierCurve(f.getPose(), g3cp1, g3Initial))
                .setConstantHeadingInterpolation(zero)
                .addParametricCallback(0.001, setMaxPower(1))
                .addPath(new BezierCurve(f.getPose(), g3Rcp1, shootPose))
                .setConstantHeadingInterpolation(zero)
                .addParametricCallback(0.5, setMaxPower(0.7))
                .build();

        cycleCornerInitial = f.pathBuilder()
                .addPath(new BezierCurve(f.getPose(), cornerIcp1, cornerInitial))
                .setConstantHeadingInterpolation(zero)
                .addParametricCallback(0.0001, setMaxPower(1))
                .addParametricCallback(0.7, setMaxPower(cornerEndPower))
                .build();

        cycleCornerEnding = f.pathBuilder()
                .addPath(new BezierLine(f.getPose(), cornerFinalStrafe))
                .setLinearHeadingInterpolation(f.getHeading(), cornerFinalStrafe.getHeading())
                .addParametricCallback(0.0001, setMaxPower(0.9))
                .addPath(new BezierLine(f.getPose(), shootPose))
                .setLinearHeadingInterpolation(f.getHeading(), zero)
                .build();

        park = f.pathBuilder()
                .addPath(new BezierLine(f.getPose(), parkPose))
                .setConstantHeadingInterpolation(zero)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case "shoot":
                if (!f.isBusy() && spinUpTimer.getElapsedTimeSeconds() > spinUpTime) {
                    if (!resetShootTimer) {
                        shootTimer.resetTimer();
                        resetShootTimer = true;
                    }
                    if (shootTimer.getElapsedTimeSeconds() > waitShootTime) {
                        r.intake.setPower(1.0);
                        r.intake.transferShoot();
                    }
                    if (shootTimer.getElapsedTimeSeconds() > shootTime) {
                        r.intake.transferBlock();
                        r.intake.setPower(1.0);
                        resetShootTimer = false;
                        set("condition");
                    }
                }
                break;

            case "condition":
                if (!f.isBusy()) {
                    r.intake.transferBlock();
                    if (cycleCount == 0) {
                        f.followPath(grabCorner, true);
                        set("cornerWait");
                        cycleCount++;
                    } else if (cycleCount == 1) {
                        f.followPath(grabThree, true);
                        set("shoot");
                        cycleCount++;
                    } else if (cycleCount > 1 && cycleCount < maxCycle) {
                        f.followPath(grabCorner2, true);
                        set("cornerWait");
                        cycleCount++;
                    } else {
                        set("park");
                    }
                }
                break;

            case "cornerWait":
                if (!f.isBusy()) {
                    r.intake.setPower(1);
                    r.intake.run();
                    r.intake.transferRun();
                    if (!resetCornerTimer) {
                        cornerTimer.resetTimer();
                        resetCornerTimer = true;
                    }
                    if (cornerTimer.getElapsedTimeSeconds() > cornerTime) {
                        resetCornerTimer = false;
                        if (cycleCount == 1) {
                            f.followPath(grabCornerReturn, true);
                            set("shoot");
                        } else if (cycleCount > 1) {
                            f.followPath(grabCornerReturn2, true);
                            set("shoot");
                        } else {
                            set("cycleCornerEndingWait");
                        }
                    }
                }
                break;

            case "cycleCornerEndingWait":
                if (!f.isBusy()) {
                    f.followPath(cycleCornerEnding, true);
                    set("shoot");
                }
                break;

            case "park":
                if (!f.isBusy()) {
                    f.followPath(park, true);
                    r.turret.stopTrack();
                    r.turret.resetTurret();
                    r.intake.stop();
                    r.shooter.stopTrack();
                    set("stopAuto");
                }
                break;

            case "stopAuto":
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

    public void set(String pathState) { this.pathState = pathState; }
    public Runnable setMaxPower(double maxPower) { return () -> f.setMaxPower(maxPower); }
    public Runnable setIntakeOn() {
        return () -> {
            r.intake.setPower(1);
            r.intake.run();
        };
    }
    public Runnable setIntakeReverse() {
        return () -> r.intake.setPower(-0.5);
    }
    public Runnable setIntakeOff() { return () -> r.intake.stop(); }

    @Override
    public void loop() {
        r.clearCache();

        f.update();
        r.update(f.getPose(), f.getVelocity());
        autonomousPathUpdate();
        Storage.lastPoseRed = f.getPose();
        Storage.lastTurretAngleRed = r.turret.getCurrentAngle();
    }

    @Override
    public void init() {
        fullTimer = new Timer();
        shootTimer = new Timer();
        cornerTimer = new Timer();
        spinUpTimer = new Timer();

        r = new Robot(hardwareMap, Alliance.RED, Teleop.FALSE, startPose);
        f = Constants.createFollower(hardwareMap);
        buildPaths();
        f.setStartingPose(startPose);

        r.intake.transferBlock();
        r.shooter.setHood(0.5);
    }

    @Override
    public void init_loop() {
        r.clearCache();
        r.turret.setAngle(0);
        telemetry.addLine("Red Far");
        telemetry.update();
    }

    @Override
    public void start() {
        fullTimer.resetTimer();
        spinUpTimer.resetTimer();
        set("shoot");
        r.intake.transferBlock();
        r.intake.run();
        r.turret.startTrack();
        r.shooter.startTrack();
    }
    @Override
    public void stop() {
        r.turret.stopTrack();
    }
}