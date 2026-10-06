package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Config
@TeleOp(name = "Turret Range Test", group = "Test Teleops")
public class TurretRangeTest extends LinearOpMode {

    // --- Dashboard inputs ---
    public static double DASHBOARD_TARGET_DEG = 0;     // edit live in dashboard to drive turret
    public static boolean USE_DASHBOARD_TARGET = false; // true = ignore gamepad, follow DASHBOARD_TARGET_DEG
    public static double GAMEPAD_RATE_DEG_PER_SEC = 60; // left-stick continuous adjust speed

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Turret turret = new Turret(hardwareMap, 0);
        // never call startTrack() — we want manual targetAngle control via setAngle()

        double targetDeg = turret.getCurrentAngle();
        double observedMin = Double.NaN;
        double observedMax = Double.NaN;
        double prevDashTarget = DASHBOARD_TARGET_DEG;
        long lastNanos = System.nanoTime();

        boolean prevA = false, prevB = false, prevX = false, prevY = false;
        boolean prevDLeft = false, prevDRight = false, prevDUp = false, prevDDown = false;

        telemetry.addLine("Turret Range Test ready.");
        telemetry.addLine("Gamepad: LStickX = continuous, dpad L/R = +/-1, dpad U/D = +/-10");
        telemetry.addLine("A = save measured MIN | B = save measured MAX | X = reset target to 0 | Y = clear");
        telemetry.addLine("Dashboard: set USE_DASHBOARD_TARGET=true and edit DASHBOARD_TARGET_DEG");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            long now = System.nanoTime();
            double dtSec = (now - lastNanos) / 1e9;
            lastNanos = now;

            // --- input source: dashboard or gamepad ---
            if (USE_DASHBOARD_TARGET) {
                targetDeg = DASHBOARD_TARGET_DEG;
            } else {
                // detect dashboard edit even when not in dashboard mode → adopt it
                if (DASHBOARD_TARGET_DEG != prevDashTarget) {
                    targetDeg = DASHBOARD_TARGET_DEG;
                }

                // continuous stick adjust
                double stick = gamepad1.left_stick_x;
                if (Math.abs(stick) > 0.05) {
                    targetDeg += stick * GAMEPAD_RATE_DEG_PER_SEC * dtSec;
                }

                // discrete dpad steps
                if (gamepad1.dpad_right && !prevDRight) targetDeg += 1;
                if (gamepad1.dpad_left  && !prevDLeft)  targetDeg -= 1;
                if (gamepad1.dpad_up    && !prevDUp)    targetDeg += 10;
                if (gamepad1.dpad_down  && !prevDDown)  targetDeg -= 10;
            }
            prevDashTarget = DASHBOARD_TARGET_DEG;

            // --- drive turret through the same path goal-tracking would use ---
            turret.setAngle(targetDeg);
            turret.update(0, 0, 0); // dx/dy/heading unused when not tracking

            double currentDeg = turret.getCurrentAngle();

            // --- min/max latch buttons ---
            if (gamepad1.a && !prevA) observedMin = currentDeg;
            if (gamepad1.b && !prevB) observedMax = currentDeg;
            if (gamepad1.x && !prevX) targetDeg = 0;
            if (gamepad1.y && !prevY) { observedMin = Double.NaN; observedMax = Double.NaN; }

            prevA = gamepad1.a; prevB = gamepad1.b;
            prevX = gamepad1.x; prevY = gamepad1.y;
            prevDLeft = gamepad1.dpad_left;   prevDRight = gamepad1.dpad_right;
            prevDUp   = gamepad1.dpad_up;     prevDDown  = gamepad1.dpad_down;

            // --- telemetry ---
            telemetry.addData("Source", USE_DASHBOARD_TARGET ? "DASHBOARD" : "GAMEPAD");
            telemetry.addData("Target deg",          targetDeg);
            telemetry.addData("Turret target deg",   turret.getTargetAngle());
            telemetry.addData("Turret current deg",  currentDeg);
            telemetry.addData("TURRET_WIRE_MIN_DEG", RobotConstants.TURRET_WIRE_MIN_DEG);
            telemetry.addData("TURRET_WIRE_MAX_DEG", RobotConstants.TURRET_WIRE_MAX_DEG);
            telemetry.addData("TURRET_SOFT_LIMIT_MARGIN_DEG", RobotConstants.TURRET_SOFT_LIMIT_MARGIN_DEG);
            telemetry.addData("TURRET_SOFT_MIN_DEG", RobotConstants.getTurretSoftMinDeg());
            telemetry.addData("TURRET_SOFT_MAX_DEG", RobotConstants.getTurretSoftMaxDeg());
            telemetry.addData("PREFERRED_FRONT_HOME_DEG", RobotConstants.getPreferredFrontHomeDeg());
            telemetry.addLine();
            telemetry.addData("Suggested soft MIN (A)", Double.isNaN(observedMin) ? "—" : String.format("%.4f", observedMin));
            telemetry.addData("Suggested soft MAX (B)", Double.isNaN(observedMax) ? "—" : String.format("%.4f", observedMax));
            telemetry.addLine();
            telemetry.addData("dbg_encoderRawDeg",   turret.dbg_encoderRawDeg);
            telemetry.addData("dbg_encoderShiftedDeg", turret.dbg_encoderShiftedDeg);
            telemetry.addData("dbg_encoderAbsDeg",   turret.dbg_encoderAbsDeg);
            telemetry.addData("dbg_inputTurretDeg",  turret.dbg_inputTurretDeg);
            telemetry.addData("dbg_adjustedDeg",     turret.dbg_adjustedDeg);
            telemetry.addData("dbg_servoDeg",        turret.dbg_servoDeg);
            telemetry.addData("dbg_servoPos",        turret.dbg_servoPos);
            telemetry.update();
        }
    }
}
