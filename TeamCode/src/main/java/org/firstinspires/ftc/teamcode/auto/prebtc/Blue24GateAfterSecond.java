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
import org.firstinspires.ftc.teamcode.util.AutoConstants;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Storage;
import org.firstinspires.ftc.teamcode.util.Teleop;

@Autonomous(name = "BLUE-24", group = "Comp Auto")
public class Blue24GateAfterSecond extends OpMode {

    private Robot r;
    private Follower f;
    private Timer fullTimer, shootTimer, rampTimer, waitTimer;
    boolean resetShootTimer = false;
    boolean resetRampTimer = false;
    boolean resetWaitTimer = false;

    private double shootTime = 0.3;
    private double rampTime = 1.6;

    private int currentCycle = 0;

    private double zero = Math.toRadians(180);

    private double lastTime = 0;

    private String pathState;

    //    private final Pose startPose = new Pose(122.46, 119.67, Math.toRadians(40.19));
    private final Pose startPose = AutoConstants.BLUE_AUTO_START_POSE;

    private final Pose shootPose = new Pose(53, 85.9, zero);

    private final Pose spike1 = new Pose(27, 82.67, zero);

//    private final Pose shootPoseAfterSpike1 = new Pose(50.58, 81.476, zero);
private final Pose shootPoseAfterSpike1 = new Pose(53.58, 85.476, zero);

    private final Pose spike2GrabControl = new Pose (46, 54.67, zero);


    private final Pose spike2 = new Pose(19, 60.3, zero);

    private final Pose returnGate = new Pose(48.97, 81.91, zero); //tangential

    private final Pose toGate = new Pose(37.2, 71.7, zero); //tangential

//    private final Pose gateIntake = new Pose(10.4, 55.3, 28.8); //constant heading
    private final Pose gateIntake = new Pose(9.9, 59.30, Math.toRadians(155));
    private final Pose gateIntake2o = new Pose(10.4, 59.65, Math.toRadians(155));
    private final Pose gateIntake3o = new Pose(10.7, 60.05, Math.toRadians(155));


    private final Pose park = new Pose(38.27, 78.76, zero); //tangential

    private double rampEndPower = 0.3;

    private Path shootPreload;
    private PathChain Spike1, grabTwo, gateIntake1, returnGateIntake1, gateIntake2, returnGateIntake2,  gateIntake3, returnGateIntake3,  gateIntake4, returnGateIntake4,  gateIntake5, returnGateIntake5, Park;

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
                .addPath(new BezierCurve(shootPoseAfterSpike1, spike2GrabControl, spike2))
                .setConstantHeadingInterpolation(zero)
                .addParametricCallback(0.1, setIntakeOn())
                .addPath(new BezierLine(spike2, returnGate))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        gateIntake1 = f.pathBuilder()
                .addPath(new BezierLine(returnGate, toGate))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(toGate, gateIntake))
                .setConstantHeadingInterpolation(gateIntake.getHeading())
//                .addParametricCallback(0.625, setMaxPower(rampEndPower))
                .build();

        returnGateIntake1 = f.pathBuilder()
                .addPath(new BezierLine(gateIntake, returnGate))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.0001, setMaxPower(1))
                .addParametricCallback(0.7, setIntakeOn())
                .build();

        gateIntake2 = f.pathBuilder()
                .addPath(new BezierLine(returnGate, toGate))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(toGate, gateIntake))
                .setConstantHeadingInterpolation(gateIntake.getHeading())
//                .addParametricCallback(0.625, setMaxPower(rampEndPower))
                .build();

        returnGateIntake2 = f.pathBuilder()
                .addPath(new BezierLine(gateIntake, returnGate))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.0001, setMaxPower(1))
                .addParametricCallback(0.7, setIntakeOn())
                .build();

        gateIntake3 = f.pathBuilder()
                .addPath(new BezierLine(returnGate, toGate))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(toGate, gateIntake))
                .setConstantHeadingInterpolation(gateIntake.getHeading())
//                .addParametricCallback(0.625, setMaxPower(rampEndPower))
                .build();

        returnGateIntake3 = f.pathBuilder()
                .addPath(new BezierLine(gateIntake, returnGate))
                .setTangentHeadingInterpolation()
                .setReversed()

                .addParametricCallback(0.0001, setMaxPower(1))
                .addParametricCallback(0.7, setIntakeOn())
                .build();

        gateIntake4 = f.pathBuilder()
                .addPath(new BezierLine(returnGate, toGate))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(toGate, gateIntake2o))
                .setConstantHeadingInterpolation(gateIntake.getHeading())
//                .addParametricCallback(0.625, setMaxPower(rampEndPower))
                .build();

        returnGateIntake4 = f.pathBuilder()
                .addPath(new BezierLine(gateIntake2o, returnGate))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.0001, setMaxPower(1))
                .addParametricCallback(0.7, setIntakeOn())
                .build();

        gateIntake5 = f.pathBuilder()
                .addPath(new BezierLine(returnGate, toGate))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(toGate, gateIntake3o))
                .setConstantHeadingInterpolation(gateIntake3o.getHeading())
//                .addParametricCallback(0.625, setMaxPower(rampEndPower))
                .build();

        returnGateIntake5 = f.pathBuilder()
                .addPath(new BezierLine(gateIntake3o, returnGate))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.0001, setMaxPower(1))
                .addParametricCallback(0.7, setIntakeOn())
                .build();

        Park = f.pathBuilder()
                .addPath(new BezierLine(returnGate, park))
                .setTangentHeadingInterpolation()
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
                        f.followPath(Spike1);
                        set("shoot");
                    } else if (currentCycle == 2) {
                        currentCycle++;
                        f.followPath(grabTwo);
                        set("rampWait");
                    } else if (currentCycle == 3) {
                        currentCycle++;
                        f.followPath(gateIntake1);
                        set("rampWait");
                    } else if (currentCycle == 4) {
                        currentCycle++;
                        f.followPath(gateIntake2);
                        set("rampWait");
                    } else if (currentCycle == 5) {
                        currentCycle++;
                        f.followPath(gateIntake3);
                        set("rampWait");
                    } else if (currentCycle == 6){
                        currentCycle++;
                        f.followPath(gateIntake4);
                        set("rampWait");
                    } else if (currentCycle == 7){
                        currentCycle++;
                        f.followPath(gateIntake5);
                        set("rampWait");
                    } else if (currentCycle == 8){
                        currentCycle++;
                        f.followPath(Park);
                        set("park");
                    }
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
                if (!f.isBusy()) {
                    if (r.intake.prepareShoot()) {
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
                            r.intake.shootComplete();
                            resetShootTimer = false;
                            if (currentCycle == 1) {
                                r.turret.startTrack();
                            }
                            set("path");
                        }
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
                        if(currentCycle == 4){
                            f.followPath((returnGateIntake1));
                        } else if(currentCycle == 5){
                            f.followPath((returnGateIntake2));
                        } else if(currentCycle == 6){
                            f.followPath((returnGateIntake3));
                        } else if(currentCycle == 7){
                            f.followPath((returnGateIntake4));
                        } else if(currentCycle == 8){
                            f.followPath((returnGateIntake5));
                        }
                        set("shoot");
                    }
                }
                break;

            case "stop":
                if(!f.isBusy()) {
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
        Storage.lastPoseBlue = f.getPose();
        Storage.lastTurretAngleBlue = r.turret.getCurrentAngle();
        telemetry.update();
    }

    @Override
    public void init() {
        fullTimer = new Timer();
        shootTimer = new Timer();
        rampTimer = new Timer();
        waitTimer = new Timer();
        r = new Robot(hardwareMap, Alliance.BLUE, Teleop.FALSE, startPose);
        r.resetPosAndIMU();
        f = Constants.createFollower(hardwareMap);
        buildPaths();
        f.setStartingPose(startPose);
        Storage.lastPoseBlue = startPose;
        Storage.lastTurretAngleBlue = r.turret.getCurrentAngle();
        r.intake.transferBlock();
        r.shooter.setHood(0.5);
        r.turret.setAngle(-3);
    }

    @Override
    public void init_loop() {
        r.clearCache();
        r.turret.setAngle(0);
        r.turret.update(0, 0, 0);
        telemetry.addLine("Blue Close 24");
        telemetry.update();
    }

    @Override
    public void start() {
        fullTimer.resetTimer();
        set("path");
        r.intake.transferBlock();
        r.intake.run();
        r.turret.setAngle(-3);
        r.shooter.startTrack();
    }

    @Override
    public void stop() { r.turret.stopTrack(); }
}
