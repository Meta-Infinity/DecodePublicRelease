package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.Teleop;


public class Intake {

    private DcMotorEx intake;
    private DcMotorEx transfer;
    private Servo latch;
    private boolean intakeFull = false;
    private boolean isTeleop;
    private boolean isIntaking = false;
    private boolean autoStopped = false;

    private DigitalChannel beam1, beam2, beam3;

    public boolean dbg_beam1Raw = false;
    public boolean dbg_beam2Raw = false;
    public boolean dbg_beam3Raw = false;

    public double dbg_beam1Avg = 0;
    public double dbg_beam2Avg = 0;
    public double dbg_beam3Avg = 0;

    public boolean dbg_beam1 = false;
    public boolean dbg_beam2 = false;
    public boolean dbg_beam3 = false;

    public int dbg_artifactCount = 0;
    public boolean dbg_beam3Sustained = false;

    private final ElapsedTime beam1BlockedTimer = new ElapsedTime();
    private final ElapsedTime beam2BlockedTimer = new ElapsedTime();
    private final ElapsedTime beam3BlockedTimer = new ElapsedTime();
    private final ElapsedTime beamAllTimer = new ElapsedTime();
    private boolean beamAllBlocked = false;
    private final ElapsedTime upstreamBlockedTimer = new ElapsedTime();

    private final ElapsedTime preReverseTimer = new ElapsedTime();
    private int preReverseState = 0;

    private double lastIntakePower = Double.NaN;
    private double lastTransferPower = Double.NaN;
    private double lastLatchPos = Double.NaN;

    private double dynamicTransferPower = 1.0;

    public Intake(HardwareMap hardwareMap, Teleop teleop) {
        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intake.setDirection(DcMotorEx.Direction.FORWARD);

        transfer = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transfer.setDirection(DcMotorEx.Direction.REVERSE);

        latch = hardwareMap.get(Servo.class, "latchservo");

        beam1 = hardwareMap.get(DigitalChannel.class, "beam1");
        beam2 = hardwareMap.get(DigitalChannel.class, "beam2");
        beam3 = hardwareMap.get(DigitalChannel.class, "beam3");
        beam1.setMode(DigitalChannel.Mode.INPUT);
        beam2.setMode(DigitalChannel.Mode.INPUT);
        beam3.setMode(DigitalChannel.Mode.INPUT);

        switch (teleop) {
            case TRUE:
                isTeleop = true;
                break;
            case FALSE:
                isTeleop = false;
                break;
            default:
                isTeleop = false;
        }
    }



    public void run(){
        setIntakeCached(1);
        isIntaking = true;
        autoStopped = false;
    }
    public void stop(){
        setIntakeCached(0);
        isIntaking = false;
    }
    public void reverse(){
        setIntakeCached(-1);
        isIntaking = false;
        dbg_artifactCount = 0;
    }
    public void transferRun(){
        setTransferCached(dbg_beam3Sustained ? RobotConstants.TRANSFER_HOLD_POWER : 1);
    }
    public void transferReverse(){
        setTransferCached(-1);
    }

    public void setPower(double power){
        setIntakeCached(power);
    }


    public void latchOpen(){
        setLatchCached(RobotConstants.LATCH_OPEN_POS);
    }

    public void setDynamicTransferPower(double power) {
        this.dynamicTransferPower = power;
    }

    public void transferShoot(){
        setTransferCached(dynamicTransferPower);
        setLatchCached(RobotConstants.LATCH_OPEN_POS);
        if (dbg_artifactCount > 0) dbg_artifactCount--;
    }

    public void transferShoot(double power){
        setTransferCached(power);
        setLatchCached(RobotConstants.LATCH_OPEN_POS);
        if (dbg_artifactCount > 0) dbg_artifactCount--;
    }

    public void transferBlock(){
        if(dbg_beam3Sustained) {
            setTransferCached(RobotConstants.TRANSFER_HOLD_POWER);
        } else if(isIntaking) {
            setTransferCached(RobotConstants.TRANSFER_IDLE_POWER);
        } else {
            setTransferCached(0);
        }
        setLatchCached(RobotConstants.LATCH_BLOCK_POS);
    }
    public void transferOpenBlock(){
        if(dbg_beam3Sustained) {
            setTransferCached(RobotConstants.TRANSFER_HOLD_POWER);
        } else if(isIntaking) {
            setTransferCached(RobotConstants.TRANSFER_IDLE_POWER);
        } else {
            setTransferCached(0);
        }
        setLatchCached(RobotConstants.LATCH_OPEN_POS);
    }

    public void transferRunSlow() {
        setTransferCached(RobotConstants.TRANSFER_CONSTANT_SPEED);
    }

    private void setIntakeCached(double power) {
        if (Double.isNaN(lastIntakePower) || Math.abs(power - lastIntakePower) > RobotConstants.INTAKE_MOTOR_THRESHOLD) {
            intake.setPower(power);
            lastIntakePower = power;
        }
    }

    private void setTransferCached(double power) {
        if (Double.isNaN(lastTransferPower) || Math.abs(power - lastTransferPower) > RobotConstants.TRANSFER_MOTOR_THRESHOLD) {
            transfer.setPower(power);
            lastTransferPower = power;
        }
    }

    private void setLatchCached(double pos) {
        if (Double.isNaN(lastLatchPos) || Math.abs(pos - lastLatchPos) > RobotConstants.LATCH_SERVO_THRESHOLD) {
            latch.setPosition(pos);
            lastLatchPos = pos;
        }
    }


    public boolean getIntakeFull(){
        return intakeFull;
    }

    public boolean getAutoStopped(){
        return autoStopped;
    }

    public int getArtifactCount(){
        return dbg_artifactCount;
    }

    public boolean isOnlyOneBall(){
        return dbg_beam3 && !dbg_beam1 && !dbg_beam2;
    }

    public boolean prepareShoot(){
        switch (preReverseState) {
            case 0:
                if (isOnlyOneBall() && RobotConstants.ONE_BALL_SHOOT_REVERSE_MS > 0) {
                    preReverseState = 1;
                    preReverseTimer.reset();
                    transferReverse();
                    return false;
                }
                preReverseState = 2;
                return true;
            case 1:
                if (preReverseTimer.milliseconds() >= RobotConstants.ONE_BALL_SHOOT_LATCH_DELAY_MS) {
                    latchOpen();
                }
                if (preReverseTimer.milliseconds() >= RobotConstants.ONE_BALL_SHOOT_REVERSE_MS) {
                    transferBlock();
                    preReverseState = 2;
                    return true;
                }
                return false;
            default:
                return true;
        }
    }

    public void shootComplete(){
        preReverseState = 0;
    }

    public double getDynamicTransferPower() {
        return dynamicTransferPower;
    }

    private boolean readBlocked(DigitalChannel ch) {
        return RobotConstants.BEAM_BLOCKED_WHEN_LOW ? !ch.getState() : ch.getState();
    }

    public void update(){
        dbg_beam1Raw = readBlocked(beam1);
        dbg_beam2Raw = readBlocked(beam2);
        dbg_beam3Raw = readBlocked(beam3);


        double a = RobotConstants.BEAM_AVG_ALPHA;
        dbg_beam1Avg = dbg_beam1Avg * (1 - a) + (dbg_beam1Raw ? 1.0 : 0.0) * a;
        dbg_beam2Avg = dbg_beam2Avg * (1 - a) + (dbg_beam2Raw ? 1.0 : 0.0) * a;
        dbg_beam3Avg = dbg_beam3Avg * (1 - a) + (dbg_beam3Raw ? 1.0 : 0.0) * a;

        boolean newB1 = dbg_beam1Avg > RobotConstants.BEAM_AVG_THRESHOLD;
        boolean newB2 = dbg_beam2Avg > RobotConstants.BEAM_AVG_THRESHOLD;
        boolean newB3 = dbg_beam3Avg > RobotConstants.BEAM_AVG_THRESHOLD;

        if (newB1 && !dbg_beam1) beam1BlockedTimer.reset();
        if (newB2 && !dbg_beam2) beam2BlockedTimer.reset();
        if (newB3 && !dbg_beam3) beam3BlockedTimer.reset();

        if (newB1 && !dbg_beam1) upstreamBlockedTimer.reset();
        if (newB2 && !dbg_beam2) upstreamBlockedTimer.reset();

        if (dbg_beam3 && !newB3
                && upstreamBlockedTimer.milliseconds() <= RobotConstants.BEAM_SEQUENCE_TIMEOUT_MS) {
            dbg_artifactCount++;
        }

        dbg_beam1 = newB1;
        dbg_beam2 = newB2;
        dbg_beam3 = newB3;

        dbg_beam3Sustained = dbg_beam3
                && beam3BlockedTimer.milliseconds() >= RobotConstants.BEAM3_TRANSFER_OFF_MS;

        boolean newAll = dbg_beam1 && dbg_beam2 && dbg_beam3;
        if (newAll && !beamAllBlocked) beamAllTimer.reset();
        beamAllBlocked = newAll;

        intakeFull  = beamAllBlocked && beamAllTimer.milliseconds() >= RobotConstants.BEAM12_INTAKE_OFF_MS;
        autoStopped = intakeFull && isIntaking;
        if (autoStopped) stop();
    }



}
