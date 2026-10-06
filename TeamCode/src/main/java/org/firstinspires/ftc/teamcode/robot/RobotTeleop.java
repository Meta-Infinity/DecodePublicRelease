package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RelocalizePosition;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.Storage;


public abstract class RobotTeleop extends LinearOpMode {

    protected Robot r;
    protected Follower f;
    protected ElapsedTime telemetryTimer = new ElapsedTime(); //timer for telemetry delay
    protected ElapsedTime loopTimer = new ElapsedTime(); //timer for loop time

    private boolean wasStalled = false;
    private boolean wasAutoStopped = false;
    private double avgHz = 0;
    private static final double HZ_SMOOTHING = 0.1;
    private boolean telemetryEnabled = RobotConstants.TELEMETRY_ON_START;

    private boolean preReversing = false;
    private boolean preReversed = false;
    private ElapsedTime preReverseTimer = new ElapsedTime();
    private boolean wasManualFeeding = false;

    private double gateHeadingRad;

    private Pose relocalizePose;

    // ── HEADING LOCK (hold gamepad1.left_bumper) ─────────────────────────────
    // While held, the robot ignores right-stick rotation and auto-rotates to
    // headingLockDegrees. The turret keeps tracking the goal automatically
    // because turret.update() recomputes its angle from the live robot heading.
    // TODO: replace the two placeholder values below with your actual headings
    private static final double HEADING_LOCK_RED      = 25;   // TODO ← red alliance target heading (degrees)
    private static final double HEADING_LOCK_BLUE     = 155.0;  // TODO ← blue alliance target heading (degrees)
    private static final double PARK_HEADING_LOCK_RED      = 0;   // TODO ← red alliance target heading (degrees)
    private static final double PARK_HEADING_LOCK_BLUE     = 180.0;

    private static final double HEADING_LOCK_MAX      = 0.5;
    private static double HEADING_LOCK_DEADBAND = 2.0;
    private static double headingKp = 0.01;
    private static double headingKs = 0.1;
    private double headingLockDegrees;
    private double parkHeadingLockDegrees;
    // ─────────────────────────────────────────────────────────────────────────

    // Gamepad state snapshots for consistent reads + edge detection
    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad currentGamepad2 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();

    protected abstract RobotConfig getConfig();

    @Override
    public void runOpMode() throws InterruptedException {
        RobotConfig config = getConfig();

        double gateHeadingDeg = (config.alliance == Alliance.RED)
                ? RobotConstants.GATE_HEADING_RED_DEG
                : RobotConstants.GATE_HEADING_BLUE_DEG;
        gateHeadingRad = Math.toRadians(gateHeadingDeg);

        double parkHeadingDeg = (config.alliance == Alliance.RED)
                ? RobotConstants.PARK_HEADING_RED_DEG
                : RobotConstants.PARK_HEADING_BLUE_DEG;
        gateHeadingRad = Math.toRadians(parkHeadingDeg);


        relocalizePose = config.relocalizePose;
        headingLockDegrees = (config.alliance == Alliance.RED) ? HEADING_LOCK_RED : HEADING_LOCK_BLUE;
        parkHeadingLockDegrees = (config.alliance == Alliance.RED) ? PARK_HEADING_LOCK_RED : PARK_HEADING_LOCK_BLUE;

//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); //comment out for comp

        r = new Robot(hardwareMap, config.alliance, config.teleop, config.startPose);

        // Restore turret absolute angle and target from storage if available
        telemetry.addData("lastTurretAngleRed", Storage.lastTurretAngleRed);
        telemetry.update();
        telemetry.addData("lastTurretAngleRed", Storage.lastTurretAngleRed);

        if (config.alliance == Alliance.RED) {
            if (!Double.isNaN(Storage.lastTurretAngleRed)) {
                r.turret.restoreState(Storage.lastTurretAngleRed);
            }
        } else {
            if (!Double.isNaN(Storage.lastTurretAngleBlue)) {
                r.turret.restoreState(Storage.lastTurretAngleBlue);
            }
        }

        // Initialization loops — Turret should NOT move here
        while (!isStarted() && !isStopRequested()) {
            r.clearCache();
            telemetry.addData("lastTurretAngleRed", Storage.lastTurretAngleRed);
            telemetry.addData("Turret Current Angle", r.round(r.turret.getCurrentAngle()));
            telemetry.addData("Turret Target Angle", r.round(r.turret.getTargetAngle()));
            telemetry.addData("Status", "Initialized - Press Play to Start");
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;

        f = Constants.createFollower(hardwareMap);

        // Retrieve the pose from storage (prioritize auto-end or previous teleop position)
        Pose startingPose = (config.alliance == Alliance.RED) ? Storage.lastPoseRed : Storage.lastPoseBlue;
        // Fallback to the config's default startPose if no previous session data exists
        if (startingPose == null) {
            startingPose = config.startPose;
        }

        f.setStartingPose(startingPose);
        f.update();

        while (opModeIsActive()) {
            // Snapshot gamepad state for consistent reads + edge detection
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            loopTimer.reset();
            update();
            drive();

            intake();
            shooter();
            turret();

            if (telemetryEnabled && telemetryTimer.milliseconds() > 300) {
                telemetry(loopTimer.milliseconds());
                telemetryTimer.reset();
            }
        }
    }

    protected void update(){
        r.clearCache();
        f.update();
        r.update(f.getPose(), f.getVelocity());

        // Continuous saving to storage during Teleop
        if (getConfig().alliance == Alliance.RED) {
            Storage.lastPoseRed = f.getPose();
            Storage.lastTurretAngleRed = r.turret.getCurrentAngle();
        } else {
            Storage.lastPoseBlue = f.getPose();
            Storage.lastTurretAngleBlue = r.turret.getCurrentAngle();
        }

        // --- Headlight Logic ---
        // Green if ramp is full, Yellow if 2 balls, Purple if 1 ball, Red otherwise
        int ballCount = r.intake.getArtifactCount();
        if (r.intake.getIntakeFull()) {
            r.lights.setColor(Lights.COLOR_GREEN);
        } else if (ballCount == 2) {
            r.lights.setColor(Lights.COLOR_YELLOW);
        } else if (ballCount == 1) {
            r.lights.setColor(Lights.COLOR_VIOLET); // Purple/Violet
        } else {
            r.lights.setColor(Lights.COLOR_RED);
        }
    }

    protected void drive(){
        if (currentGamepad1.y && !previousGamepad1.y) {
            RelocalizePosition gate = (getConfig().alliance == Alliance.RED) ? RelocalizePosition.RED_GATE : RelocalizePosition.BLUE_GATE;
            f.setPose(gate.defaultPose);
            r.turret.resetOffsetDegrees();
            r.shooter.resetOffsetRPM();
        }

        if(currentGamepad1.left_trigger >0.1){
            r.turret.resetOffsetDegrees();
            r.shooter.resetOffsetRPM();
            r.kfOn();
        } else{
            r.kfOff();
        }
        //meow

//        double y  = -currentGamepad1.left_stick_y;
//        double x  =  currentGamepad1.left_stick_x;
//        double rx =  currentGamepad1.right_stick_x;

        double rawY  = -currentGamepad1.left_stick_y;
        double rawX  =  currentGamepad1.left_stick_x;
        double y  = Math.abs(rawY)  > 0.05 ? rawY  : 0;
        double x  = Math.abs(rawX)  > 0.05 ? rawX  : 0;

        double rx;
        if (currentGamepad1.left_bumper) {
            double current = Math.toDegrees(f.getPose().getHeading());
            double error   = Math.IEEEremainder(current - headingLockDegrees, 360);
            if (Math.abs(error) < HEADING_LOCK_DEADBAND) {
                rx = 0;
            } else {
                rx = Math.max(-HEADING_LOCK_MAX, Math.min(HEADING_LOCK_MAX, error * headingKp + headingKs * Math.signum(error)));
            }
        } else if (currentGamepad2.left_bumper) {
            double current = Math.toDegrees(f.getPose().getHeading());
            double error = Math.IEEEremainder(current - parkHeadingLockDegrees, 360);
            if (Math.abs(error) < HEADING_LOCK_DEADBAND) {
                rx = 0;
            } else {
                rx = Math.max(-HEADING_LOCK_MAX, Math.min(HEADING_LOCK_MAX, error * headingKp + headingKs * Math.signum(error)));
            }
        }
        else {
            rx = currentGamepad1.right_stick_x;
        }


        r.drivetrain.drive(y, x, rx, 1);
    }

    protected void intake() {

//        //stop intake (rising edge)
//        if (currentGamepad1.left_trigger > 0.5 && !(previousGamepad1.left_trigger > 0.5)) {
//            r.intake.stop();
//            preReversed = false;
//            preReversing = false;
//        }

        //start intake (rising edge)
        if (currentGamepad1.a && !previousGamepad1.a) {
            r.intake.run();
            preReversed = false;
            preReversing = false;
        }

        //reverse (rising edge)
        if(currentGamepad1.b && !previousGamepad1.b){
            r.intake.reverse();
            r.intake.transferReverse();
            preReversed = false;
            preReversing = false;
        }


        //haptics -> intake motor stalling
        if(r.intake.getIntakeFull() && !wasStalled){
            gamepad1.rumble(500);
            gamepad2.rumble(500);
            wasStalled = true;
        }
        if(wasStalled && !r.intake.getIntakeFull()){
            wasStalled = false;
        }

        // progressive RGB gradient: red→green based on artifact count (only while loading, not after auto-stop)
        /*if(!r.intake.getAutoStopped() && r.intake.getArtifactCount() > 0) {
            double fraction = Math.min(
                    (double) r.intake.getArtifactCount() / RobotConstants.TARGET_BALL_COUNT, 1.0);
            r.lights.setGradient(fraction);
        }*/

        //haptics + pre-reverse on stall auto-stop (ramp full)
        if(r.intake.getAutoStopped() && !wasAutoStopped){
            gamepad1.rumble(1000);
            gamepad2.rumble(1000);
            wasAutoStopped = true;
        }
        if(wasAutoStopped && !r.intake.getAutoStopped()){
            wasAutoStopped = false;
        }

        //pre-reverse state machine — runs independently each loop
        if(preReversing) {
            if(preReverseTimer.milliseconds() >= RobotConstants.SHOOT_LATCH_DELAY_MS) {
                r.intake.latchOpen();
            }
            if(preReverseTimer.milliseconds() >= RobotConstants.SHOOT_REVERSE_MS) {
                preReversing = false;
                preReversed = true;
                r.intake.transferBlock();
            }
        }

//        if (currentGamepad1.right_bumper) {
//        //transfer shoot
////        if (currentGamepad1.right_bumper || currentGamepad1.x) {
////            RobotConstants.TRANSFER_IDLE_POWER = .8 * (currentGamepad1.x ? .5 : 1);
//            if (preReversed) {
//                //already pre-reversed — skip to shooting immediately
//                preReversed = false;
//                r.shooter.isShooting();
//                r.intake.transferShoot();
//                r.intake.run();
//            } else if (!preReversing && !r.shooter.getShootingStatus()) {
//                //normal reverse sequence
//                startPreReverse();
//            }
//            //if preReversing, let it finish — next loop will see preReversed=true
//            if (!preReversing && r.shooter.getShootingStatus()) {
//                //sustain shooting while held (caching prevents redundant writes)
//                r.intake.transferShoot();
//                r.intake.run();
//            }
//        } else if (currentGamepad1.right_trigger > 0.01) {
//            //manual feed — hold to run transfer + intake, no latch, no reverse
//            wasManualFeeding = true;
//            r.intake.run();
//            r.intake.transferRun();
//        } else if (!currentGamepad1.b) {
//            //idle state
//            r.shooter.notShooting();
//            if (wasManualFeeding) {
//                r.intake.stop();
//                wasManualFeeding = false;
//                startPreReverse();
//            }
//            if (!preReversing) {
//                r.intake.transferBlock();
//            }
//        }

        if (currentGamepad1.right_bumper || currentGamepad1.x) {
            double shootPower = currentGamepad1.x && !currentGamepad1.right_bumper ? 0.3 : r.intake.getDynamicTransferPower();

            if (preReversed) {
                preReversed = false;
                r.shooter.isShooting();
                r.intake.transferShoot(shootPower);
                r.intake.run();
            } else if (!preReversing && !r.shooter.getShootingStatus()) {
                startPreReverse();
            }
            if (!preReversing && r.shooter.getShootingStatus()) {
                r.intake.transferShoot(shootPower);
                r.intake.run();
            }
        } else if (currentGamepad1.right_trigger > 0.01) {
            wasManualFeeding = true;
            r.intake.run();
            r.intake.transferRun();
        } else if (!currentGamepad1.b) {
            r.shooter.notShooting();
            if (wasManualFeeding) {
                r.intake.stop();
                wasManualFeeding = false;
                startPreReverse();
            }
            if (!preReversing) {
                r.intake.transferBlock();
            }
        }
    }

    private void startPreReverse() {
        if (!preReversed && !preReversing) {
            preReversing = true;
            preReverseTimer.reset();
            r.intake.transferReverse();
        }
    }

    protected void shooter(){
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            r.shooter.startTrack();
        }
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            r.shooter.stopTrack();
        }
    }

    protected void turret(){
        boolean driver1SwitchGoal = currentGamepad1.share && !previousGamepad1.share;
        boolean driver2SwitchGoal = currentGamepad2.share && !previousGamepad2.share;
        if (driver1SwitchGoal || driver2SwitchGoal) {
            r.switchGoal();
        }

        if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
            r.turret.startTrack();
            r.sotmOn();
        }
        if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
            r.turret.stopTrack();
            r.turret.resetTurret();
            r.sotmOff();
        }

        if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
            r.turret.addOffSetDegrees(2);
        }
        if(currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
            r.turret.addOffSetDegrees(-2);
        }
        if(currentGamepad2.dpad_left && !previousGamepad2.dpad_left){
            r.turret.addOffSetDegrees(2);
        }
        if(currentGamepad2.dpad_right && !previousGamepad2.dpad_right){
            r.turret.addOffSetDegrees(-2);
        }

        if(currentGamepad2.x && !previousGamepad2.x){
            r.turret.addOffSetDegrees(1.5);
        }
        if(currentGamepad2.b && !previousGamepad2.b){
            r.turret.addOffSetDegrees(-1.5);
        }


        if(currentGamepad2.dpad_up && !previousGamepad2.dpad_up){
            r.shooter.addOffsetRPM(10);
        }
        if(currentGamepad2.dpad_down && !previousGamepad2.dpad_down){
            r.shooter.addOffsetRPM(-10);
        }
    }

    protected void telemetry(double loopTime) {
        Pose pose = f.getPose();

        telemetry.addData("Shooter RPM", r.shooter.getCurrentRPM());
//        // telemetry.addData("LL distance inch", r.getLLDistance());
        telemetry.addData("Real distance inch", r.shooter.getDistance());
        telemetry.addData("Target", r.getTargetName());
//        telemetry.addData("Hood Position", r.shooter.getHoodPosition());
//        telemetry.addData("Auto Shooting", r.shooter.isAutoShooting());
        telemetry.addData("Turret Angle", " Current: " + r.round(r.turret.getCurrentAngle()) + " Target: " + r.round(r.turret.getTargetAngle()));
        telemetry.addData("Robot Pose", " x: " + r.round(pose.getX()) + " y: " + r.round(pose.getY()) + " h: " + Math.toDegrees(r.round(pose.getHeading())));
//        Pose llPose = r.getLLPose();
//        telemetry.addData("2. LL Pose", "X: " + r.round(llPose.getX()) + " Y: " + r.round(llPose.getY()));
//        Pose llRaw = r.getLLRawPose();
//        telemetry.addData("2a. LL Raw MT2 (in)", "X: " + r.round(llRaw.getX()) + " Y: " + r.round(llRaw.getY()));
//        Pose llRawMT1 = r.getLLRawPoseMT1();
//        telemetry.addData("2b. LL Raw MT1 (in)", "X: " + r.round(llRawMT1.getX()) + " Y: " + r.round(llRawMT1.getY()));
            telemetry.addData("Fusion Pose", " x: " + r.round(r.getFusionPose().getX()) + " y: " + r.round(r.getFusionPose().getY()) + "h: " + Math.toDegrees(r.round(r.getFusionPose().getHeading())));
        telemetry.addData("LL Pose", " x: " + r.round(r.getLLPose().getX()) + " y: " + r.round(r.getLLPose().getY()) + "h: " + Math.toDegrees(r.round(r.getLLPose().getHeading())));
        telemetry.addData("Raw LL Pose", " x: " + r.round(r.getLLRawPose().getX()) + " y: " + r.round(r.getLLRawPose().getY()) + "h: " + Math.toDegrees(r.round(r.getLLRawPose().getHeading())));
//        telemetry.addData("SOTM = ", r.getSOTM());
//        telemetry.addData("Track Pose", " x: " + r.round(r.getTrackPose().getX()) + " y: " + r.round(r.getTrackPose().getY()));
//        telemetry.addData("Velocity", " x: " + r.round(f.getVelocity().getXComponent()) + " y: " + r.round(f.getVelocity().getYComponent()));
//        telemetry.addData("Loop Time ms: ", loopTime);
        double hz = loopTime > 0 ? 1000.0 / loopTime : 0;
        avgHz = avgHz * (1 - HZ_SMOOTHING) + hz * HZ_SMOOTHING;
        telemetry.addData("Loop Hz: ", r.round(avgHz));
//
//        // --- Intake/Ramp beams ---
//        telemetry.addData("Beam 1 (top)",    r.intake.dbg_beam1 + " raw:" + r.intake.dbg_beam1Raw + " avg:" + r.round(r.intake.dbg_beam1Avg));
//        telemetry.addData("Beam 2 (mid)",    r.intake.dbg_beam2 + " raw:" + r.intake.dbg_beam2Raw + " avg:" + r.round(r.intake.dbg_beam2Avg));
//        telemetry.addData("Beam 3 (bottom)", r.intake.dbg_beam3 + " raw:" + r.intake.dbg_beam3Raw + " avg:" + r.round(r.intake.dbg_beam3Avg));
//        telemetry.addData("Beam3 Sustained (transfer off)", r.intake.dbg_beam3Sustained);
//        telemetry.addData("Artifact Count",  r.intake.getArtifactCount());
//        telemetry.addData("Intake Full",     r.intake.getIntakeFull());
//        telemetry.addData("Auto Stopped (ramp full)", r.intake.getAutoStopped());
//
//        // --- Turret debug (all sent to FTC Dashboard via MultipleTelemetry) ---
//        telemetry.addData("T_gearRatio", RobotConstants.GEAR_RATIO);
//        telemetry.addData("T_servoRangeDeg", RobotConstants.SERVO_RANGE_DEG);
//        telemetry.addData("T_maxTurretDeg", RobotConstants.MAX_TURRET_DEG);
//        telemetry.addData("T_wireMinDeg", RobotConstants.TURRET_WIRE_MIN_DEG);
//        telemetry.addData("T_wireMaxDeg", RobotConstants.TURRET_WIRE_MAX_DEG);
//        telemetry.addData("T_softMarginDeg", RobotConstants.TURRET_SOFT_LIMIT_MARGIN_DEG);
//        telemetry.addData("T_softMinTargetDeg", RobotConstants.getTurretSoftMinDeg());
//        telemetry.addData("T_softMaxTargetDeg", RobotConstants.getTurretSoftMaxDeg());
//        telemetry.addData("T_frontHomeDeg", RobotConstants.getPreferredFrontHomeDeg());
//        telemetry.addData("T_zeroOffset", RobotConstants.ZERO_OFFSET_DEG);
//        telemetry.addData("T_inputTurretDeg", r.round(r.turret.dbg_inputTurretDeg));
//        telemetry.addData("T_adjustedDeg", r.round(r.turret.dbg_adjustedDeg));
//        telemetry.addData("T_servoDeg", r.round(r.turret.dbg_servoDeg));
//        telemetry.addData("T_servoPos", r.round(r.turret.dbg_servoPos));
//        telemetry.addData("T_xToGoal", r.round(r.turret.dbg_xToGoal));
//        telemetry.addData("T_yToGoal", r.round(r.turret.dbg_yToGoal));
//        telemetry.addData("T_robotHeadingDeg", r.round(r.turret.dbg_robotHeadingDeg));
//        telemetry.addData("T_headingToGoalDeg", r.round(r.turret.dbg_headingToGoalDeg));
//        telemetry.addData("T_rawTargetAngle", r.round(r.turret.dbg_rawTargetTurretAngle));
//        telemetry.addData("T_clampedTargetAngle", r.round(r.turret.dbg_clampedTargetTurretAngle));
//        telemetry.addData("T_offsetDeg", r.round(r.turret.getOffsetDegrees()));
//        telemetry.addData("T_encoderRawDeg",  r.round(r.turret.dbg_encoderRawDeg));
//        telemetry.addData("T_encoderShiftedDeg", r.round(r.turret.dbg_encoderShiftedDeg));
//        telemetry.addData("T_encoderAbsDeg",  r.round(r.turret.dbg_encoderAbsDeg));
//        telemetry.addData("T_pidError",       r.round(r.turret.dbg_pidError));
//        telemetry.addData("T_pidOutput",      r.round(r.turret.dbg_pidOutput));
//        telemetry.addData("T_commandedPower", r.round(r.turret.dbg_commandedPower));
//
//        telemetry.addData("Lights Blinking", r.lights.isBlinking());

        telemetry.update();
    }
}
