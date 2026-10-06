package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.RobotConstants;


public class Lights {

    private Servo rgb;
    private Servo headlight;

    private ElapsedTime blinkTimer = new ElapsedTime();
    private int blinksRemaining = 0;
    private boolean blinkOn = false;
    private double lastRgbPos = Double.NaN;
    private double lastHeadlightPos = Double.NaN;

    public static final double COLOR_OFF    = 0.0;
    public static final double COLOR_RED    = 0.277;
    public static final double COLOR_ORANGE = 0.333;
    public static final double COLOR_YELLOW = 0.388;
    public static final double COLOR_SAGE   = 0.444;
    public static final double COLOR_GREEN  = 0.500;
    public static final double COLOR_AZURE  = 0.555;
    public static final double COLOR_BLUE   = 0.611;
    public static final double COLOR_INDIGO = 0.666;
    public static final double COLOR_VIOLET = 0.722;
    public static final double COLOR_WHITE  = 1.0;

    public Lights(HardwareMap hardwareMap) {
        rgb = hardwareMap.get(Servo.class, "rgb");
        headlight = hardwareMap.get(Servo.class, "headlight");
        rgb.setPosition(COLOR_OFF);
        headlight.setPosition(0);
    }

    public void setColor(double position) {
        setRgbCached(position);
    }

    public void setGradient(double fraction) {
        fraction = Math.max(0, Math.min(1, fraction));
        double pos = COLOR_RED + fraction * (COLOR_GREEN - COLOR_RED);
        setRgbCached(pos);
    }

    public void startBlink() {
        blinksRemaining = RobotConstants.BLINK_COUNT;
        blinkOn = true;
        blinkTimer.reset();
        setHeadlightCached(1);
    }

    public void off() {
        setRgbCached(COLOR_OFF);
        setHeadlightCached(0);
        blinksRemaining = 0;
        blinkOn = false;
    }

    public boolean isBlinking() {
        return blinksRemaining > 0;
    }

    public void update() {
        if (blinksRemaining <= 0) return;

        if (blinkOn && blinkTimer.milliseconds() >= RobotConstants.BLINK_ON_MS) {
            setHeadlightCached(0);
            blinkOn = false;
            blinkTimer.reset();
        } else if (!blinkOn && blinkTimer.milliseconds() >= RobotConstants.BLINK_OFF_MS) {
            blinksRemaining--;
            if (blinksRemaining > 0) {
                setHeadlightCached(1);
                blinkOn = true;
                blinkTimer.reset();
            }
        }
    }

    private void setRgbCached(double pos) {
        if (Double.isNaN(lastRgbPos) || Math.abs(pos - lastRgbPos) > RobotConstants.LIGHT_SERVO_THRESHOLD) {
            rgb.setPosition(pos);
            lastRgbPos = pos;
        }
    }

    private void setHeadlightCached(double pos) {
        if (Double.isNaN(lastHeadlightPos) || Math.abs(pos - lastHeadlightPos) > RobotConstants.LIGHT_SERVO_THRESHOLD) {
            headlight.setPosition(pos);
            lastHeadlightPos = pos;
        }
    }
}
