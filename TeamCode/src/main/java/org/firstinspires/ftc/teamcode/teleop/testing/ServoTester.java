package org.firstinspires.ftc.teamcode.teleop.testing;

    import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "Servo Tester", group = "Test Teleops")
public class ServoTester extends LinearOpMode {

    public static boolean STATE_REVERSED = true;

    public static boolean SERVO_1_ENABLED = true;
    public static String SERVO_1_NAME = "latchservo";
    public static double SERVO_1_FORWARD_POS = 0.615;
    public static double SERVO_1_REVERSE_POS = 0.54;
    public static boolean SERVO_1_SWAP = false;

    public static boolean SERVO_2_ENABLED = false;
    public static String SERVO_2_NAME = "";
    public static double SERVO_2_FORWARD_POS = 0.0;
    public static double SERVO_2_REVERSE_POS = 1.0;
    public static boolean SERVO_2_SWAP = false;
    public static boolean SERVO_2_MATCH_SERVO_1 = false;

    private final ServoSlot servo1Slot = new ServoSlot();
    private final ServoSlot servo2Slot = new ServoSlot();
    private boolean lastAPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Servo Tester ready");
        telemetry.addLine("Configure servo names and booleans in FTCDashboard");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a && !lastAPressed) {
                STATE_REVERSED = !STATE_REVERSED;
            }
            lastAPressed = gamepad1.a;

            servo1Slot.refresh(SERVO_1_ENABLED, SERVO_1_NAME);
            servo2Slot.refresh(SERVO_2_ENABLED, SERVO_2_NAME);

            double servo1Command = getConfiguredPosition(
                    SERVO_1_FORWARD_POS,
                    SERVO_1_REVERSE_POS,
                    SERVO_1_SWAP
            );

            double servo2Command = SERVO_2_MATCH_SERVO_1
                    ? servo1Command
                    : getConfiguredPosition(
                            SERVO_2_FORWARD_POS,
                            SERVO_2_REVERSE_POS,
                            SERVO_2_SWAP
                    );

            servo1Slot.apply(servo1Command);
            servo2Slot.apply(servo2Command);

            telemetry.addData("State Reversed", STATE_REVERSED);
            telemetry.addData("Servo 1", servo1Slot.getStatus());
            telemetry.addData("Servo 1 Forward", Range.clip(SERVO_1_FORWARD_POS, 0.0, 1.0));
            telemetry.addData("Servo 1 Reverse", Range.clip(SERVO_1_REVERSE_POS, 0.0, 1.0));
            telemetry.addData("Servo 1 Swap", SERVO_1_SWAP);
            telemetry.addData("Servo 1 Command", servo1Slot.getLastCommand());

            telemetry.addData("Servo 2", servo2Slot.getStatus());
            telemetry.addData("Servo 2 Mode", SERVO_2_MATCH_SERVO_1 ? "MATCH_SERVO_1" : "INDEPENDENT");
            telemetry.addData("Servo 2 Forward", Range.clip(SERVO_2_FORWARD_POS, 0.0, 1.0));
            telemetry.addData("Servo 2 Reverse", Range.clip(SERVO_2_REVERSE_POS, 0.0, 1.0));
            telemetry.addData("Servo 2 Swap", SERVO_2_SWAP);
            telemetry.addData("Servo 2 Command", servo2Slot.getLastCommand());
            telemetry.update();
        }
    }

    private double getConfiguredPosition(double forwardPosition, double reversePosition, boolean swap) {
        double activeForward = swap ? reversePosition : forwardPosition;
        double activeReverse = swap ? forwardPosition : reversePosition;
        return Range.clip(STATE_REVERSED ? activeReverse : activeForward, 0.0, 1.0);
    }

    private final class ServoSlot {
        private Servo servo;
        private boolean enabled;
        private String configuredName = "";
        private String warning = "disabled";
        private double lastCommand = Double.NaN;

        void refresh(boolean enabled, String rawName) {
            String normalizedName = normalize(rawName);
            boolean needsRebind = this.enabled != enabled || !configuredName.equals(normalizedName);

            this.enabled = enabled;

            if (!needsRebind) {
                return;
            }

            configuredName = normalizedName;
            servo = null;
            warning = enabled ? "not bound" : "disabled";
            lastCommand = Double.NaN;

            if (!enabled) {
                return;
            }

            if (configuredName.isEmpty()) {
                warning = "missing hardware name";
                return;
            }

            try {
                servo = hardwareMap.get(Servo.class, configuredName);
                warning = "bound";
            } catch (RuntimeException e) {
                warning = "missing in hardware map";
            }
        }

        void apply(double position) {
            if (!enabled || servo == null) {
                lastCommand = Double.NaN;
                return;
            }

            double clippedPosition = Range.clip(position, 0.0, 1.0);
            servo.setPosition(clippedPosition);
            lastCommand = clippedPosition;
        }

        String getStatus() {
            String namePart = configuredName.isEmpty() ? "<blank>" : configuredName;
            return "enabled=" + enabled + " name=" + namePart + " status=" + warning;
        }

        double getLastCommand() {
            return lastCommand;
        }

        private String normalize(String name) {
            return name == null ? "" : name.trim();
        }
    }
}
