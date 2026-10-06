package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.controlSystems.LookUpTable;
import org.firstinspires.ftc.teamcode.controlSystems.PIDController;
import org.firstinspires.ftc.teamcode.util.RobotConstants;


public class Shooter {

    private DcMotorEx shooterMotor1, shooterMotor2, encoderMotor;
    private Servo hood1;

    private LookUpTable storage;
    private PIDController pid;

    private boolean isShooting = false;
    private boolean autoShooting = false;
    private boolean controlSystemOn = false;

    private double targetRPM = 0;
    private double hoodPosition = 0;

    //CONSTANTS
    private final double TICKS_PER_REV = 28;

    private double distance = 0;
    private double lastShooterPower = Double.NaN;
    private double lastHoodPos = Double.NaN;
    private double cachedRPM = 0;

    public Shooter(HardwareMap hardwareMap, LookUpTable storage) {
        hood1 = hardwareMap.get(Servo.class, "hood1");
//        hood1 = hardwareMap.get(Servo.class, "hood2");
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterleft");
        shooterMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterright");
        shooterMotor2.setDirection(DcMotorEx.Direction.FORWARD);
        shooterMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        encoderMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");

        pid = new PIDController(
                RobotConstants.SHOOTER_P,
                RobotConstants.SHOOTER_I,
                RobotConstants.SHOOTER_D,
                RobotConstants.SHOOTER_F,
                true
        );

        this.storage = storage;
    }


    public void addOffsetRPM(double rpm){
        RobotConstants.OFFSET_RPM += rpm;
    }
    public void addOffsetHood(double angle){
        RobotConstants.OFFSET_HOOD += angle;
    }

    public void setRPM(double rpm){
        targetRPM = (rpm + RobotConstants.OFFSET_RPM) * RobotConstants.SPEED_SCALE;
    }
    public void setHood(double position){
        hoodPosition = Range.clip(position + RobotConstants.OFFSET_HOOD, 0, 1);
        if (Double.isNaN(lastHoodPos) || Math.abs(hoodPosition - lastHoodPos) > RobotConstants.HOOD_SERVO_THRESHOLD) {
            hood1.setPosition(hoodPosition);
            lastHoodPos = hoodPosition;
        }
    }
    public double getCurrentRPM(){
        return cachedRPM;
    }


    public void isShooting(){
        isShooting = true;
    }

    public void notShooting(){
        isShooting = false;
    }
    public boolean getShootingStatus(){
        return isShooting;
    }



    public void startTrack(){
        autoShooting = true;
        controlSystemOn = true;
    }
    public void stopTrack(){
        autoShooting = false;
        controlSystemOn = false;
        setRPM(-1);
    }

    public double getDistance(){
        return distance;
    }

    public double getHoodPosition(){
        return hoodPosition;
    }

    public boolean isAutoShooting(){
        return autoShooting;
    }

    public void resetOffsetRPM() {
        RobotConstants.OFFSET_RPM = 0;
    }


    //need to run this constantly
    public void update(double distanceToGoal, double voltage) {
        distance = distanceToGoal;
        if(autoShooting){
            setRPM(storage.getRPM(distance));
            setHood(storage.getHood(distance));
        }
        cachedRPM = -encoderMotor.getVelocity() * 60.0 / TICKS_PER_REV;
        double currentRPM = cachedRPM;


        if (targetRPM <= 0) {
            setShooterPowerCached(0);
            controlSystemOn = false;
        } else{
            controlSystemOn = true;
        }



        if(controlSystemOn) {
            if (!isShooting) {
                double pidOutput = pid.update(targetRPM, currentRPM, voltage);
                setShooterPowerCached(pidOutput);
            } else {
                if (currentRPM < targetRPM - RobotConstants.DEADBAND_RPM) {
                    setShooterPowerCached(RobotConstants.BANG_VELOCITY);
                } else {
                    setShooterPowerCached(0);
                }
            }
        }

    }

    private void setShooterPowerCached(double power) {
        if (Double.isNaN(lastShooterPower) || Math.abs(power - lastShooterPower) > RobotConstants.SHOOTER_MOTOR_THRESHOLD) {
            shooterMotor1.setPower(power);
            shooterMotor2.setPower(power);
            lastShooterPower = power;
        }
    }

    public void stop() {
        targetRPM = 0;
        setShooterPowerCached(0);
        pid.reset();
    }


}
