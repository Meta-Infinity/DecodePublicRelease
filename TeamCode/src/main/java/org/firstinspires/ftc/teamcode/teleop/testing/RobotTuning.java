package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Selectable tuner for servo/turret calibration.
 * During init, navigate the menu (A = select, dpad = move). After selecting a tuner, press start.
 *
 * Tuners:
 *  - "Turret Zero Offset Finder": rotate turret by hand to physical 0°, press A to capture the
 *    raw encoder reading as the value to paste into RobotConstants.ENCODER_ZERO_DEG.
 *  - "Turret Range Finder": command the turret around manually and save measured soft limits.
 *  - "Latch Position Tester": jog latch servo and toggle between BLOCK and OPEN constants.
 *  - "Hood Min/Max Tester": jog hood servo, press A to save MIN, B to save MAX.
 */
@Config
@TeleOp(name = "Robot Tuning", group = "Test Teleops")
public class RobotTuning extends SelectableOpMode {

    private static double wrap360(double deg) {
        return ((deg % 360) + 360) % 360;
    }

    public RobotTuning() {
        super("Select a tuner", s -> {
            s.add("Turret Zero Offset Finder", TurretZeroOffsetFinder::new);
            s.add("Turret Range Finder",       TurretRangeFinder::new);
            s.add("Latch Position Tester",     LatchTester::new);
            s.add("Hood Min/Max Tester",       HoodTester::new);
        });
    }

    // ───────────────────────────────────────────────────────────────────────────
    // Turret Zero Offset Finder
    //   Move turret by hand to point straight forward (the physical 0° heading).
    //   Press A to capture: the displayed "captured raw" is what to paste into
    //   RobotConstants.ENCODER_ZERO_DEG. Press B to clear the capture.
    // ───────────────────────────────────────────────────────────────────────────
    public static class TurretZeroOffsetFinder extends OpMode {
        private AnalogInput encoder;
        private MultipleTelemetry tele;
        private double capturedRaw = Double.NaN;
        private boolean prevA = false, prevB = false;

        @Override public void init() {
            encoder = hardwareMap.get(AnalogInput.class, "elcencoder");
            tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            tele.addLine("Turret Zero Offset Finder");
            tele.addLine("Rotate turret by hand to point straight forward.");
            tele.addLine("A = capture | B = clear");
            tele.update();
        }

        @Override public void loop() {
            double raw = wrap360((encoder.getVoltage() / RobotConstants.ENCODER_WRAP_VOLTAGE) * 360.0);

            if (gamepad1.a && !prevA) capturedRaw = raw;
            if (gamepad1.b && !prevB) capturedRaw = Double.NaN;
            prevA = gamepad1.a; prevB = gamepad1.b;

            tele.addData("Raw encoder deg",        raw);
            tele.addData("Wrap voltage",           RobotConstants.ENCODER_WRAP_VOLTAGE);
            tele.addData("Current ENCODER_ZERO_DEG", RobotConstants.ENCODER_ZERO_DEG);
            tele.addLine();
            if (Double.isNaN(capturedRaw)) {
                tele.addLine("No capture. Press A at the 0° position.");
            } else {
                tele.addLine("---- PASTE INTO RobotConstants ----");
                tele.addData("ENCODER_ZERO_DEG", capturedRaw);
                tele.addLine("-----------------------------------");
            }
            tele.update();
        }
    }

    // ───────────────────────────────────────────────────────────────────────────
    // Turret Range Finder
    //   LStickX = continuous adjust. Dpad L/R = +/-1 deg. Dpad U/D = +/-10 deg.
    //   A = save measured MIN, B = save measured MAX, X = reset target to 0, Y = clear saves.
    // ───────────────────────────────────────────────────────────────────────────
    public static class TurretRangeFinder extends OpMode {
        private Turret turret;
        private MultipleTelemetry tele;
        private double targetDeg = 0;
        private double observedMin = Double.NaN;
        private double observedMax = Double.NaN;
        private double prevDashTarget = TurretRangeTest.DASHBOARD_TARGET_DEG;
        private long lastNanos = 0;

        private boolean prevA = false, prevB = false, prevX = false, prevY = false;
        private boolean prevDLeft = false, prevDRight = false, prevDUp = false, prevDDown = false;

        @Override public void init() {
            turret = new Turret(hardwareMap, 0);
            tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            targetDeg = turret.getCurrentAngle();
            lastNanos = System.nanoTime();

            tele.addLine("Turret Range Finder");
            tele.addLine("LStickX = continuous, dpad L/R = +/-1, dpad U/D = +/-10");
            tele.addLine("A = save MIN | B = save MAX | X = target 0 | Y = clear");
            tele.addLine("Dashboard: USE_DASHBOARD_TARGET + DASHBOARD_TARGET_DEG also work");
            tele.update();
        }

        @Override public void loop() {
            long now = System.nanoTime();
            double dtSec = (now - lastNanos) / 1e9;
            lastNanos = now;

            if (TurretRangeTest.USE_DASHBOARD_TARGET) {
                targetDeg = TurretRangeTest.DASHBOARD_TARGET_DEG;
            } else {
                if (TurretRangeTest.DASHBOARD_TARGET_DEG != prevDashTarget) {
                    targetDeg = TurretRangeTest.DASHBOARD_TARGET_DEG;
                }

                double stick = gamepad1.left_stick_x;
                if (Math.abs(stick) > 0.05) {
                    targetDeg += stick * TurretRangeTest.GAMEPAD_RATE_DEG_PER_SEC * dtSec;
                }

                if (gamepad1.dpad_right && !prevDRight) targetDeg += 1;
                if (gamepad1.dpad_left  && !prevDLeft)  targetDeg -= 1;
                if (gamepad1.dpad_up    && !prevDUp)    targetDeg += 10;
                if (gamepad1.dpad_down  && !prevDDown)  targetDeg -= 10;
            }
            prevDashTarget = TurretRangeTest.DASHBOARD_TARGET_DEG;

            turret.setAngle(targetDeg);
            turret.update(0, 0, 0);

            double currentDeg = turret.getCurrentAngle();

            if (gamepad1.a && !prevA) observedMin = currentDeg;
            if (gamepad1.b && !prevB) observedMax = currentDeg;
            if (gamepad1.x && !prevX) targetDeg = 0;
            if (gamepad1.y && !prevY) { observedMin = Double.NaN; observedMax = Double.NaN; }

            prevA = gamepad1.a; prevB = gamepad1.b;
            prevX = gamepad1.x; prevY = gamepad1.y;
            prevDLeft = gamepad1.dpad_left;   prevDRight = gamepad1.dpad_right;
            prevDUp   = gamepad1.dpad_up;     prevDDown  = gamepad1.dpad_down;

            tele.addData("Source", TurretRangeTest.USE_DASHBOARD_TARGET ? "DASHBOARD" : "GAMEPAD");
            tele.addData("Target deg", targetDeg);
            tele.addData("Turret target deg", turret.getTargetAngle());
            tele.addData("Turret current deg", currentDeg);
            tele.addData("TURRET_WIRE_MIN_DEG", RobotConstants.TURRET_WIRE_MIN_DEG);
            tele.addData("TURRET_WIRE_MAX_DEG", RobotConstants.TURRET_WIRE_MAX_DEG);
            tele.addData("TURRET_SOFT_LIMIT_MARGIN_DEG", RobotConstants.TURRET_SOFT_LIMIT_MARGIN_DEG);
            tele.addData("TURRET_SOFT_MIN_DEG", RobotConstants.getTurretSoftMinDeg());
            tele.addData("TURRET_SOFT_MAX_DEG", RobotConstants.getTurretSoftMaxDeg());
            tele.addData("PREFERRED_FRONT_HOME_DEG", RobotConstants.getPreferredFrontHomeDeg());
            tele.addLine();
            tele.addData("Suggested soft MIN (A)", Double.isNaN(observedMin) ? "—" : String.format("%.4f", observedMin));
            tele.addData("Suggested soft MAX (B)", Double.isNaN(observedMax) ? "—" : String.format("%.4f", observedMax));
            tele.addLine();
            tele.addData("dbg_encoderRawDeg", turret.dbg_encoderRawDeg);
            tele.addData("dbg_encoderShiftedDeg", turret.dbg_encoderShiftedDeg);
            tele.addData("dbg_encoderAbsDeg", turret.dbg_encoderAbsDeg);
            tele.addData("dbg_inputTurretDeg", turret.dbg_inputTurretDeg);
            tele.addData("dbg_adjustedDeg", turret.dbg_adjustedDeg);
            tele.addData("dbg_servoDeg", turret.dbg_servoDeg);
            tele.addData("dbg_servoPos", turret.dbg_servoPos);
            tele.update();
        }
    }

    // ───────────────────────────────────────────────────────────────────────────
    // Latch Position Tester
    //   A = jump to LATCH_BLOCK_POS, B = jump to LATCH_OPEN_POS.
    //   LB/RB = nudge -/+ 0.005. Stick Y = continuous nudge.
    //   X = capture current as new BLOCK suggestion, Y = capture as OPEN suggestion.
    // ───────────────────────────────────────────────────────────────────────────
    public static class LatchTester extends OpMode {
        private Servo latch;
        private MultipleTelemetry tele;
        private double pos = 0.5;
        private double suggestedBlock = Double.NaN;
        private double suggestedOpen = Double.NaN;
        private boolean prevA = false, prevB = false, prevX = false, prevY = false;
        private boolean prevLB = false, prevRB = false;

        @Override public void init() {
            latch = hardwareMap.get(Servo.class, "latchservo");
            tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            pos = RobotConstants.LATCH_BLOCK_POS;
            latch.setPosition(pos);
            tele.addLine("Latch Position Tester");
            tele.addLine("A=BLOCK  B=OPEN  LB/RB=nudge ±0.005  LStickY=continuous");
            tele.addLine("X=save current as BLOCK suggestion, Y=save as OPEN");
            tele.update();
        }

        @Override public void loop() {
            if (gamepad1.a  && !prevA)  pos = RobotConstants.LATCH_BLOCK_POS;
            if (gamepad1.b  && !prevB)  pos = RobotConstants.LATCH_OPEN_POS;
            if (gamepad1.left_bumper  && !prevLB) pos -= 0.005;
            if (gamepad1.right_bumper && !prevRB) pos += 0.005;

            double stick = -gamepad1.left_stick_y; // up = positive
            if (Math.abs(stick) > 0.05) pos += stick * 0.002; // ~0.2/sec at full stick

            pos = Range.clip(pos, 0.0, 1.0);

            if (gamepad1.x && !prevX) suggestedBlock = pos;
            if (gamepad1.y && !prevY) suggestedOpen  = pos;

            prevA = gamepad1.a; prevB = gamepad1.b;
            prevX = gamepad1.x; prevY = gamepad1.y;
            prevLB = gamepad1.left_bumper; prevRB = gamepad1.right_bumper;

            latch.setPosition(pos);

            tele.addData("Position",          pos);
            tele.addData("LATCH_BLOCK_POS",   RobotConstants.LATCH_BLOCK_POS);
            tele.addData("LATCH_OPEN_POS",    RobotConstants.LATCH_OPEN_POS);
            tele.addLine();
            if (!Double.isNaN(suggestedBlock)) tele.addData("Suggested LATCH_BLOCK_POS (X)", suggestedBlock);
            if (!Double.isNaN(suggestedOpen))  tele.addData("Suggested LATCH_OPEN_POS  (Y)", suggestedOpen);
            tele.update();
        }
    }

    // ───────────────────────────────────────────────────────────────────────────
    // Hood Min/Max Tester
    //   LStickY = continuous nudge. LB/RB = step ±0.005.
    //   A = save MIN, B = save MAX, X = clear.
    //   Telemetry shows raw position (no OFFSET_HOOD applied).
    // ───────────────────────────────────────────────────────────────────────────
    public static class HoodTester extends OpMode {
        private Servo hood;
        private MultipleTelemetry tele;
        private double pos = 0.5;
        private double savedMin = Double.NaN;
        private double savedMax = Double.NaN;
        private boolean prevA = false, prevB = false, prevX = false;
        private boolean prevLB = false, prevRB = false;

        @Override public void init() {
            hood = hardwareMap.get(Servo.class, "hood1");
            tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            hood.setPosition(pos);
            tele.addLine("Hood Min/Max Tester");
            tele.addLine("LStickY=continuous  LB/RB=step ±0.005");
            tele.addLine("A=save MIN  B=save MAX  X=clear");
            tele.update();
        }

        @Override public void loop() {
            if (gamepad1.left_bumper  && !prevLB) pos -= 0.005;
            if (gamepad1.right_bumper && !prevRB) pos += 0.005;
            double stick = -gamepad1.left_stick_y;
            if (Math.abs(stick) > 0.05) pos += stick * 0.002;
            pos = Range.clip(pos, 0.0, 1.0);

            if (gamepad1.a && !prevA) savedMin = pos;
            if (gamepad1.b && !prevB) savedMax = pos;
            if (gamepad1.x && !prevX) { savedMin = Double.NaN; savedMax = Double.NaN; }

            prevA = gamepad1.a; prevB = gamepad1.b; prevX = gamepad1.x;
            prevLB = gamepad1.left_bumper; prevRB = gamepad1.right_bumper;

            hood.setPosition(pos);

            tele.addData("Position (raw, no OFFSET_HOOD)", pos);
            tele.addData("OFFSET_HOOD",                    RobotConstants.OFFSET_HOOD);
            tele.addLine();
            tele.addData("Saved MIN (A)", Double.isNaN(savedMin) ? "—" : String.format("%.4f", savedMin));
            tele.addData("Saved MAX (B)", Double.isNaN(savedMax) ? "—" : String.format("%.4f", savedMax));
            tele.update();
        }
    }
}
