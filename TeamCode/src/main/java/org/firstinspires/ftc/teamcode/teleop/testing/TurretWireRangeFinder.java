package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Config
@TeleOp(name = "Turret Wire Range Finder", group = "Test Teleops")
public class TurretWireRangeFinder extends LinearOpMode {

    public static double HOME_TARGET_DEG = 0;
    public static double HOME_TOLERANCE_DEG = 3;
    public static double HOME_SETTLE_MS = 350;
    public static double HOME_TIMEOUT_MS = 5000;

    public static double LEFT_SEARCH_POWER = -0.12;
    public static double RIGHT_SEARCH_POWER = 0.12;
    public static double SEARCH_STARTUP_MS = 300;
    public static double STALL_WINDOW_DEG = 5;
    public static double STALL_SETTLE_MS = 450;
    public static double SEARCH_TIMEOUT_MS = 7000;
    public static boolean RETURN_TO_ZERO_AFTER_SCAN = true;

    private MultipleTelemetry tele;
    private Turret turret;

    private String phase = "INIT";
    private double currentDeg = Double.NaN;
    private double leftLimitDeg = Double.NaN;
    private double rightLimitDeg = Double.NaN;
    private double observedMinDeg = Double.NaN;
    private double observedMaxDeg = Double.NaN;
    private String summary = "Waiting to start";

    @Override
    public void runOpMode() throws InterruptedException {
        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turret = new Turret(hardwareMap, 0);
        currentDeg = turret.getCurrentAngle();

        tele.addLine("Turret Wire Range Finder ready");
        tele.addLine("On start: home to 0, search left wire limit, re-home, search right wire limit.");
        tele.addLine("Dashboard params: LEFT_SEARCH_POWER / RIGHT_SEARCH_POWER / STALL_WINDOW_DEG");
        tele.update();

        waitForStart();
        if (isStopRequested()) return;

        boolean homedBeforeLeft = moveToTarget(HOME_TARGET_DEG, "Homing to 0 before left search");
        if (!homedBeforeLeft) {
            summary = "Failed to settle at 0 before left search";
            finishAndReport();
            return;
        }

        leftLimitDeg = findWireLimit(LEFT_SEARCH_POWER, "Finding left limit");

        boolean homedBeforeRight = moveToTarget(HOME_TARGET_DEG, "Returning to 0 before right search");
        if (!homedBeforeRight) {
            summary = "Left limit found, but failed to settle back at 0 before right search";
            finishAndReport();
            return;
        }

        rightLimitDeg = findWireLimit(RIGHT_SEARCH_POWER, "Finding right limit");

        if (RETURN_TO_ZERO_AFTER_SCAN && opModeIsActive()) {
            moveToTarget(HOME_TARGET_DEG, "Returning to 0 after scan");
        }

        observedMinDeg = minIgnoringNaN(leftLimitDeg, rightLimitDeg);
        observedMaxDeg = maxIgnoringNaN(leftLimitDeg, rightLimitDeg);
        summary = "Scan complete";
        finishAndReport();
    }

    private boolean moveToTarget(double targetDeg, String nextPhase) {
        phase = nextPhase;
        ElapsedTime timeout = new ElapsedTime();
        ElapsedTime inToleranceTimer = new ElapsedTime();
        boolean inTolerance = false;

        while (opModeIsActive() && timeout.milliseconds() < HOME_TIMEOUT_MS) {
            turret.setAngle(targetDeg);
            turret.update(0, 0, 0);
            currentDeg = turret.getCurrentAngle();
            double errorDeg = targetDeg - currentDeg;

            if (Math.abs(errorDeg) <= HOME_TOLERANCE_DEG) {
                if (!inTolerance) {
                    inTolerance = true;
                    inToleranceTimer.reset();
                }
                if (inToleranceTimer.milliseconds() >= HOME_SETTLE_MS) {
                    renderTelemetry(errorDeg, 0, Double.NaN, Double.NaN, "Settled");
                    return true;
                }
            } else {
                inTolerance = false;
            }

            renderTelemetry(errorDeg, 0, Double.NaN, Double.NaN, "Homing");
            idle();
        }

        turret.stop();
        return false;
    }

    private double findWireLimit(double searchPower, String nextPhase) {
        phase = nextPhase;
        ElapsedTime timeout = new ElapsedTime();
        ElapsedTime stillTimer = new ElapsedTime();
        double startDeg = turret.updateEncoderOnly();
        currentDeg = startDeg;
        double anchorDeg = startDeg;
        double minSeenDeg = startDeg;
        double maxSeenDeg = startDeg;

        while (opModeIsActive() && timeout.milliseconds() < SEARCH_TIMEOUT_MS) {
            turret.setManualPower(searchPower);
            currentDeg = turret.updateEncoderOnly();

            minSeenDeg = Math.min(minSeenDeg, currentDeg);
            maxSeenDeg = Math.max(maxSeenDeg, currentDeg);

            if (Math.abs(currentDeg - anchorDeg) > STALL_WINDOW_DEG) {
                anchorDeg = currentDeg;
                stillTimer.reset();
            }

            boolean pastStartup = timeout.milliseconds() >= SEARCH_STARTUP_MS;
            boolean stalled = pastStartup && stillTimer.milliseconds() >= STALL_SETTLE_MS;

            renderTelemetry(HOME_TARGET_DEG - currentDeg, searchPower, minSeenDeg, maxSeenDeg,
                    stalled ? "Stall detected" : "Searching");

            if (stalled) {
                break;
            }

            idle();
        }

        turret.stop();
        currentDeg = turret.updateEncoderOnly();

        double negativeTravel = startDeg - minSeenDeg;
        double positiveTravel = maxSeenDeg - startDeg;
        return positiveTravel >= negativeTravel ? maxSeenDeg : minSeenDeg;
    }

    private void finishAndReport() {
        turret.stop();
        while (opModeIsActive()) {
            currentDeg = turret.updateEncoderOnly();
            renderTelemetry(HOME_TARGET_DEG - currentDeg, 0, leftLimitDeg, rightLimitDeg, summary);
            idle();
        }
    }

    private void renderTelemetry(double errorDeg, double power, double rangeA, double rangeB, String detail) {
        tele.addData("Phase", phase);
        tele.addData("Detail", detail);
        tele.addData("Current deg", currentDeg);
        tele.addData("Target/home deg", HOME_TARGET_DEG);
        tele.addData("Error deg", errorDeg);
        tele.addData("Commanded power", power);
        tele.addLine();
        tele.addData("Left limit deg", formatMaybe(leftLimitDeg));
        tele.addData("Right limit deg", formatMaybe(rightLimitDeg));
        tele.addData("Observed min deg", formatMaybe(minIgnoringNaN(observedMinDeg, minIgnoringNaN(leftLimitDeg, rightLimitDeg))));
        tele.addData("Observed max deg", formatMaybe(maxIgnoringNaN(observedMaxDeg, maxIgnoringNaN(leftLimitDeg, rightLimitDeg))));
        tele.addData("Suggested TURRET_SOFT_MIN_DEG", formatMaybe(getSuggestedSoftMin()));
        tele.addData("Suggested TURRET_SOFT_MAX_DEG", formatMaybe(getSuggestedSoftMax()));
        tele.addLine();
        tele.addData("Range sample A", formatMaybe(rangeA));
        tele.addData("Range sample B", formatMaybe(rangeB));
        tele.addData("dbg_encoderRawDeg", turret.dbg_encoderRawDeg);
        tele.addData("dbg_encoderShiftedDeg", turret.dbg_encoderShiftedDeg);
        tele.addData("dbg_encoderAbsDeg", turret.dbg_encoderAbsDeg);
        tele.addData("dbg_commandedPower", turret.dbg_commandedPower);
        tele.update();
    }

    private double getSuggestedSoftMin() {
        if (Double.isNaN(leftLimitDeg) && Double.isNaN(rightLimitDeg)) return Double.NaN;
        return minIgnoringNaN(leftLimitDeg, rightLimitDeg) + RobotConstants.TURRET_SOFT_LIMIT_MARGIN_DEG;
    }

    private double getSuggestedSoftMax() {
        if (Double.isNaN(leftLimitDeg) && Double.isNaN(rightLimitDeg)) return Double.NaN;
        return maxIgnoringNaN(leftLimitDeg, rightLimitDeg) - RobotConstants.TURRET_SOFT_LIMIT_MARGIN_DEG;
    }

    private double minIgnoringNaN(double a, double b) {
        if (Double.isNaN(a)) return b;
        if (Double.isNaN(b)) return a;
        return Math.min(a, b);
    }

    private double maxIgnoringNaN(double a, double b) {
        if (Double.isNaN(a)) return b;
        if (Double.isNaN(b)) return a;
        return Math.max(a, b);
    }

    private String formatMaybe(double value) {
        return Double.isNaN(value) ? "—" : String.format("%.4f", value);
    }
}
