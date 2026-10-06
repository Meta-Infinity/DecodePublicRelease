package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class RobotConstants {

    // Turret
    public static double SERVO_RANGE_DEG = 348.0392156863;
    public static double GEAR_RATIO = 1.1818181818181819;
    public static double MAX_TURRET_DEG = 411.3190730838091;
    public static double MIN_TURRET_DEG = 0;
    public static double ZERO_OFFSET_DEG = 205.65953654190454;
    public static boolean INVERT = true;
    public static double TURRET_FORWARD_OFFSET_DEG = 0;
    public static boolean SLEW_RATE_ENABLED = true;
    public static double SLEW_RATE_DEG_PER_SEC = 720;
    public static double TURRET_SLEW_MAX_DT_SEC = 0.05;
    public static double TURRET_ANG_VEL_LEAD_SEC = 0.05;

    // Turret Physical Offset (Inches)
    public static double TURRET_X_OFFSET = 0;
    public static double TURRET_Y_OFFSET = -0.19685;

    // Turret PID + encoder
    public static double ENCODER_WRAP_VOLTAGE = 3.2;            // wrap the analog encoder at 3.2 V, not the noisy top-end
    public static double ENCODER_WRAP_OFFSET_DEG = 180;         // positive seam shift to move the unwrap boundary away from common aim angles
    //    public static double ENCODER_ZERO_DEG = 182.3625;  // raw encoder deg when turret faces forward on the 3.2 V wrap scale
    public static double ENCODER_ZERO_DEG = 181.35;  // raw encoder deg when turret faces forward on the 3.2 V wrap scale

    // Dual PID Tuning
    public static double TURRET_P_FAR = 0.013;
    public static double TURRET_I_FAR = 0;
    public static double TURRET_D_FAR = 0.0016;
    public static double TURRET_F_FAR = 0.09;
    public static double TURRET_V_FF_FAR = 0; // Velocity Feedforward multiplier for target velocity

    public static double TURRET_P_CLOSE = 0.0028;
    public static double TURRET_I_CLOSE = 0;
    public static double TURRET_D_CLOSE = 0.0009;
    public static double TURRET_F_CLOSE = 0.09;
    public static double TURRET_V_FF_CLOSE = 0;

    public static double TURRET_PID_SWITCH_THRESHOLD_DEG = 15;
    public static double TURRET_PID_SWITCH_HYSTERESIS_DEG = 2; // band around switch point to prevent close/far chatter

    public static double TURRET_MAX_POWER = 0.5;
    public static boolean TURRET_PID_OUTPUT_INVERT = true;
    public static double TURRET_WIRE_MIN_DEG = -455;
    public static double TURRET_WIRE_MAX_DEG = 95; //97.975;
    public static double TURRET_SOFT_LIMIT_MARGIN_DEG = 5;

    // Shooter
    public static double SHOOTER_P = 0.006;
    public static double SHOOTER_I = 0;
    public static double SHOOTER_D = 0;
    public static double SHOOTER_F = 1;
    public static double DEADBAND_RPM = 15;
    public static double BANG_VELOCITY = 1;
    public static double OFFSET_RPM = 0;
    public static double OFFSET_HOOD = 0.14; //-0.4;
    public static double MANUAL_HOOD_POSITION = 0.5;
    public static double SPEED_SCALE = 1;

    // Testing Constants for Interp
    public static double TEST_SHOOTER_RPM = 0;
    public static double TEST_HOOD_POSITION = 0.5;

    // Intake
    public static double LATCH_BLOCK_POS = 0.65;
    public static double LATCH_OPEN_POS = 0.78;
    public static double TRANSFER_IDLE_POWER = 0.8;
    public static double TRANSFER_HOLD_POWER = 0;
    public static double TRANSFER_CONSTANT_SPEED = 0.2;
    public static double SHOOT_REVERSE_MS = 0;
    public static double SHOOT_LATCH_DELAY_MS = 0;
    public static double ONE_BALL_SHOOT_REVERSE_MS = 70;
    public static double ONE_BALL_SHOOT_LATCH_DELAY_MS = 0;

    // Break beams (ramp ball detection — beam1 = front of intake, beam3 = bottom near transfer/latch)
    public static boolean BEAM_BLOCKED_WHEN_LOW = true;    // Adafruit default; flip if wiring inverted
    public static double  BEAM_AVG_ALPHA = 0.3;            // EMA smoothing on raw blocked samples (0..1, higher = faster)
    public static double  BEAM_AVG_THRESHOLD = 0.5;        // smoothed value > threshold => beam considered blocked
    public static double  BEAM3_TRANSFER_OFF_MS = 80;      // beam3 sustained this long => transfer drops to TRANSFER_HOLD_POWER
    public static double  BEAM12_INTAKE_OFF_MS = 100;      // beam1 AND beam2 sustained this long => auto-stop intake (ramp full)
    public static double  BEAM_SEQUENCE_TIMEOUT_MS = 130; //old 400  // beam1/2 must have triggered within this window before beam3 trailing edge counts
    public static int     TARGET_BALL_COUNT = 3;           // used for LED gradient scaling

    // Field
//    public static double RED_GOAL_X = 91;
//    public static double RED_GOAL_Y = 111;
    public static double RED_GOAL_X = 134;
    public static double RED_GOAL_Y = 138;
    public static double BLUE_GOAL_X = 8;
    public static double BLUE_GOAL_Y = 137;
    public static double RED_MIDDLE_X = 62;
    public static double RED_MIDDLE_Y = 126;
    public static double BLUE_MIDDLE_X = 80;
    public static double BLUE_MIDDLE_Y = 126;

    // Auto
    // Lights
    public static double BLINK_ON_MS = 50;
    public static double BLINK_OFF_MS = 50;
    public static int BLINK_COUNT = 10;

    // Kalman
    public static double KALMAN_Q = 0.003;
    public static double KALMAN_R = 8;


    // Telemetry
    public static boolean TELEMETRY_ON_START = true;

    // Gate Alignment
    public static double GATE_HEADING_RED_DEG  = 25.7831;
    public static double GATE_HEADING_BLUE_DEG = 180 - 25.7831; // 154.2169

    public static double PARK_HEADING_RED_DEG  = 0;
    public static double PARK_HEADING_BLUE_DEG = 180; // 154.2169

    public static double GATE_HEADING_P = 0.8;
    public static double GATE_HEADING_DEADBAND_DEG = 2.0;

    // Hardware Write Caching Thresholds
    public static double DRIVE_MOTOR_THRESHOLD = 0.02;
    public static double SHOOTER_MOTOR_THRESHOLD = 0.005;
    public static double INTAKE_MOTOR_THRESHOLD = 0.02;
    public static double TRANSFER_MOTOR_THRESHOLD = 0.02;
    public static double TURRET_SERVO_THRESHOLD = 0.001;
    public static double HOOD_SERVO_THRESHOLD = 0.002;
    public static double LATCH_SERVO_THRESHOLD = 0.001;
    public static double LIGHT_SERVO_THRESHOLD = 0.001;

    public static double getTurretSoftMinDeg() {
        return TURRET_WIRE_MIN_DEG + TURRET_SOFT_LIMIT_MARGIN_DEG;
    }

    public static double getTurretSoftMaxDeg() {
        return TURRET_WIRE_MAX_DEG - TURRET_SOFT_LIMIT_MARGIN_DEG;
    }

    public static double getPreferredFrontHomeDeg() {
        double softMin = getTurretSoftMinDeg();
        double softMax = getTurretSoftMaxDeg();

        int kMin = (int) Math.ceil(softMin / 360.0);
        int kMax = (int) Math.floor(softMax / 360.0);

        if (kMin > kMax) {
            return 360.0 * Math.round(((softMin + softMax) / 2.0) / 360.0);
        }

        double bestCandidate = 360.0 * kMin;
        double bestClearance = Double.NEGATIVE_INFINITY;
        final double CLEARANCE_TIE_EPS = 1e-6;

        for (int k = kMin; k <= kMax; k++) {
            double candidate = 360.0 * k;
            double clearance = Math.min(candidate - softMin, softMax - candidate);
            boolean better = clearance > bestClearance + CLEARANCE_TIE_EPS
                    || (Math.abs(clearance - bestClearance) <= CLEARANCE_TIE_EPS
                    && Math.abs(candidate) < Math.abs(bestCandidate));
            if (better) {
                bestClearance = clearance;
                bestCandidate = candidate;
            }
        }

        return bestCandidate;
    }
}
