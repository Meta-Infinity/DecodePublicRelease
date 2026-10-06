package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.RobotConstants;

@TeleOp(name = "Turret Test", group = "Test teleops")
public class TurretTest extends LinearOpMode {

    private static final double ENCODER_ZERO_DEG = 255.0545; // raw encoder deg when turret faces front

    private Servo servo1, servo2, servo3;
    private AnalogInput encoder;
    private double turretDeg = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        encoder = hardwareMap.get(AnalogInput.class, "elcencoder");

        setTurretPosition(0);

        telemetry.addData("Status", "Ready. Dpad right +90, left -90, up +45, down reset to 0");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double lowerBound = -(RobotConstants.MAX_TURRET_DEG - RobotConstants.ZERO_OFFSET_DEG) + 10;
            double upperBound = (RobotConstants.MAX_TURRET_DEG - RobotConstants.ZERO_OFFSET_DEG) - 5;
            if (gamepad1.dpad_right && !prevDpadRight) {
                turretDeg = Range.clip(turretDeg + 90, lowerBound, upperBound);
            }
            if (gamepad1.dpad_left && !prevDpadLeft) {
                turretDeg = Range.clip(turretDeg - 90, lowerBound, upperBound);
            }
            if (gamepad1.dpad_up && !prevDpadUp) {
                turretDeg = Range.clip(turretDeg + 45, lowerBound, upperBound);
            }
            if (gamepad1.dpad_down && !prevDpadDown) {
                turretDeg = 0;
            }

            prevDpadRight = gamepad1.dpad_right;
            prevDpadLeft = gamepad1.dpad_left;
            prevDpadUp = gamepad1.dpad_up;
            prevDpadDown = gamepad1.dpad_down;

            double adjustedDeg = turretDeg + RobotConstants.ZERO_OFFSET_DEG;
            adjustedDeg = Range.clip(adjustedDeg, RobotConstants.MIN_TURRET_DEG, RobotConstants.MAX_TURRET_DEG);
            double servoPos = (adjustedDeg / RobotConstants.GEAR_RATIO) / RobotConstants.SERVO_RANGE_DEG;
            servoPos = Range.clip(servoPos, 0.0, 1.0);

            setTurretPosition(turretDeg);

            telemetry.addData("turret target deg", turretDeg);
            telemetry.addData("adjusted deg (with offset)", adjustedDeg);
            telemetry.addData("servo pos (A)", servoPos);
            telemetry.addData("servo pos (B/C)", 1.0 - servoPos);
            telemetry.addData("servo angle deg", adjustedDeg / RobotConstants.GEAR_RATIO);
            double encoderDeg = (encoder.getVoltage() / encoder.getMaxVoltage()) * 360.0;
            double encoderZeroed = encoderDeg - ENCODER_ZERO_DEG;
            // wrap to [-180, 180)
            encoderZeroed = ((encoderZeroed + 180) % 360 + 360) % 360 - 180;
            telemetry.addData("encoder voltage", encoder.getVoltage());
            telemetry.addData("encoder raw deg", encoderDeg);
            telemetry.addData("encoder zeroed deg", encoderZeroed);
            telemetry.addData("controls", "dpad R +90 | L -90 | U +45 | D reset");
            telemetry.update();
        }
    }

    private boolean prevDpadRight, prevDpadLeft, prevDpadUp, prevDpadDown;

    private void setTurretPosition(double turretDeg) {
        double adjustedDeg = turretDeg + RobotConstants.ZERO_OFFSET_DEG;
        adjustedDeg = Range.clip(adjustedDeg, RobotConstants.MIN_TURRET_DEG, RobotConstants.MAX_TURRET_DEG);

        double servoPos = (adjustedDeg / RobotConstants.GEAR_RATIO) / RobotConstants.SERVO_RANGE_DEG;
        servoPos = Range.clip(servoPos, 0.0, 1.0);

        servo1.setPosition(servoPos);
        servo2.setPosition(1.0 - servoPos);
        servo3.setPosition(1.0 - servoPos);
    }
}
