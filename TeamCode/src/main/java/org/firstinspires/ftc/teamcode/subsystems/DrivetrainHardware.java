package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.RobotConstants;

public class DrivetrainHardware {

    private DcMotorEx leftFrontMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightBackMotor;

    private double lastLF = Double.NaN;
    private double lastRF = Double.NaN;
    private double lastLB = Double.NaN;
    private double lastRB = Double.NaN;

    public DrivetrainHardware(HardwareMap hardwareMap) {
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "backRight");

        leftFrontMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorEx.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    public void drive(double y, double x, double rx, double speed) {
        double frontLeftPower = (y + x + rx) * speed;
        double frontRightPower = (y - x - rx) * speed;
        double backLeftPower = (y - x + rx) * speed;
        double backRightPower = (y + x - rx) * speed;

        double maxPower = Math.max(1.0, Math.max(
                Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));

        frontLeftPower /= maxPower;
        frontRightPower /= maxPower;
        backLeftPower /= maxPower;
        backRightPower /= maxPower;

        if (Double.isNaN(lastLF) || Math.abs(frontLeftPower - lastLF) > RobotConstants.DRIVE_MOTOR_THRESHOLD) {
            leftFrontMotor.setPower(frontLeftPower);
            lastLF = frontLeftPower;
        }
        if (Double.isNaN(lastRF) || Math.abs(frontRightPower - lastRF) > RobotConstants.DRIVE_MOTOR_THRESHOLD) {
            rightFrontMotor.setPower(frontRightPower);
            lastRF = frontRightPower;
        }
        if (Double.isNaN(lastLB) || Math.abs(backLeftPower - lastLB) > RobotConstants.DRIVE_MOTOR_THRESHOLD) {
            leftBackMotor.setPower(backLeftPower);
            lastLB = backLeftPower;
        }
        if (Double.isNaN(lastRB) || Math.abs(backRightPower - lastRB) > RobotConstants.DRIVE_MOTOR_THRESHOLD) {
            rightBackMotor.setPower(backRightPower);
            lastRB = backRightPower;
        }
    }

}