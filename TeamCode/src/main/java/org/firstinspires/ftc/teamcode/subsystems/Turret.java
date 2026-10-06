package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.controlSystems.PIDController;
import org.firstinspires.ftc.teamcode.util.RobotConstants;


public class Turret {

    private CRServo servo1, servo2, servo3;
    private AnalogInput encoder;
    private PIDController pidClose, pidFar;

    private boolean reverseServo1 = false;
    private boolean reverseServo2 = true;
    private boolean reverseServo3 = false;

    private boolean isTracking = false;

    private boolean useClosePid = false;

    private double targetAngle = 0;
    private double currentCommandedAngle = 0;
    private double encoderWrapOffsetDeg;

    private double offsetDegrees = 0;

    private double startingAngle = 0;

    private ElapsedTime slewTimer = new ElapsedTime();

    private double lastShiftedEncoderDeg = Double.NaN;
    private double absEncoderDeg = 0;   // continuous turret frame deg, can be <- 180 or >180

    private double lastPower = Double.NaN;

    public double dbg_inputTurretDeg = 0;
    public double dbg_adjustedDeg = 0;
    public double dbg_servoDeg = 0;
    public double dbg_servoPos = 0;
    public double dbg_xToGoal = 0;
    public double dbg_yToGoal = 0;
    public double dbg_robotHeadingDeg = 0;
    public double dbg_headingToGoalDeg = 0;
    public double dbg_rawTargetTurretAngle = 0;
    public double dbg_clampedTargetTurretAngle = 0;
    public double dbg_angVelDegPerSec = 0;
    public double dbg_angVelLeadDeg = 0;

    public double dbg_encoderRawDeg = 0;
    public double dbg_encoderShiftedDeg = 0;
    public double dbg_encoderAbsDeg = 0;
    public double dbg_pidError = 0;
    public double dbg_pidOutput = 0;
    public double dbg_commandedPower = 0;

    public Turret(HardwareMap hardwareMap, double startingAngle) {
        this(hardwareMap, startingAngle, RobotConstants.ENCODER_WRAP_OFFSET_DEG);
    }

    public Turret(HardwareMap hardwareMap, double startingAngle, double encoderWrapOffsetDeg) {
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");
        servo3 = hardwareMap.get(CRServo.class, "servo3");

        servo1.setDirection(reverseServo1 ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        servo2.setDirection(reverseServo2 ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        servo3.setDirection(reverseServo3 ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        encoder = hardwareMap.get(AnalogInput.class, "elcencoder");

        pidClose = new PIDController(
                RobotConstants.TURRET_P_CLOSE,
                RobotConstants.TURRET_I_CLOSE,
                RobotConstants.TURRET_D_CLOSE,
                RobotConstants.TURRET_F_CLOSE,
                false // position mode, not velocity
        );

        pidFar = new PIDController(
                RobotConstants.TURRET_P_FAR,
                RobotConstants.TURRET_I_FAR,
                RobotConstants.TURRET_D_FAR,
                RobotConstants.TURRET_F_FAR,
                false
        );

        this.startingAngle = startingAngle;
        this.encoderWrapOffsetDeg = encoderWrapOffsetDeg;

        // seed encoder unwrap from current physical position (assumes turret is within ±180° of forward at boot)
        seedEncoder();

        targetAngle = RobotConstants.getPreferredFrontHomeDeg();
        setTurretPosition(targetAngle);
    }


    public void startTrack(){
        isTracking = true;
    }
    public void stopTrack(){
        isTracking = false;
    }


    public void setAngle(double degrees){
        targetAngle = degrees + offsetDegrees;
    }

    public void resetTurret(){
        targetAngle = RobotConstants.getPreferredFrontHomeDeg();
    }

    public double getCurrentAngle(){
        return absEncoderDeg + startingAngle;
    }

    public void restoreState(double storedAngle) {
        double currentRaw = readRawEncoderDeg();
        double currentZeroed = currentRaw - RobotConstants.ENCODER_ZERO_DEG;
        //wrap to [-180, 180)
        currentZeroed = ((currentZeroed + 180) % 360 + 360) % 360 - 180;

        //current total angle (unwrapped) assuming it's in the base rotation loop
        double currentTotal = currentZeroed + startingAngle;

        //find the number of full rotations 'k' needed to get closest to storedAngle
        double diff = storedAngle - currentTotal;
        double k = Math.round(diff / 360.0);

        //restore absolute state with the correct rotation loop
        absEncoderDeg = currentZeroed + k * 360.0;
        lastShiftedEncoderDeg = readShiftedEncoderDeg(currentRaw); //sync the delta tracker

        targetAngle = absEncoderDeg + startingAngle;
        currentCommandedAngle = targetAngle;
        slewTimer.reset();

        dbg_encoderAbsDeg = absEncoderDeg;
        dbg_encoderRawDeg = currentRaw;
    }

    public void setAbsoluteAngle(double angle) {
        absEncoderDeg = angle - startingAngle;
    }

    public double getTargetAngle(){
        return targetAngle;
    }

    public void addOffSetDegrees(double degrees){
        offsetDegrees += degrees;
    }

    public double getOffsetDegrees(){
        return offsetDegrees;
    }

    public void resetOffsetDegrees() {
        offsetDegrees = 0;
    }

    public double updateEncoderOnly() {
        double encoderDeg = readEncoderUnwrapped();
        currentCommandedAngle = encoderDeg;
        targetAngle = encoderDeg;

        dbg_inputTurretDeg = encoderDeg;
        return getCurrentAngle();
    }

    public void setManualPower(double power) {
        pidClose.reset();
        pidFar.reset();
        dbg_pidError = 0;
        dbg_pidOutput = 0;
        setServoPowerCached(Range.clip(power, -1.0, 1.0));
    }

    public void stop() {
        setManualPower(0);
    }

    private double wrap360(double deg) {
        return ((deg % 360) + 360) % 360;
    }

    private double readRawEncoderDeg() {
        return wrap360((encoder.getVoltage() / RobotConstants.ENCODER_WRAP_VOLTAGE) * 360.0);
    }

    private double readShiftedEncoderDeg(double rawDeg) {
        double shifted = wrap360(rawDeg + encoderWrapOffsetDeg);
        dbg_encoderShiftedDeg = shifted;
        return shifted;
    }

    private void seedEncoder() {
        double raw = readRawEncoderDeg();
        double zeroed = raw - RobotConstants.ENCODER_ZERO_DEG;
        //wrap to [-180, 180)
        zeroed = ((zeroed + 180) % 360 + 360) % 360 - 180;
        absEncoderDeg = zeroed;
        lastShiftedEncoderDeg = readShiftedEncoderDeg(raw);
        dbg_encoderRawDeg = raw;
        dbg_encoderAbsDeg = absEncoderDeg;
    }

    private double readEncoderUnwrapped() {
        double raw = readRawEncoderDeg();
        dbg_encoderRawDeg = raw;
        double shifted = readShiftedEncoderDeg(raw);
        double delta = shifted - lastShiftedEncoderDeg;
        if (delta > 180)  delta -= 360;
        if (delta < -180) delta += 360;
        //accumulate in turret-frame; shifting the seam does not change the physical delta.
        absEncoderDeg += delta;
        lastShiftedEncoderDeg = shifted;
        dbg_encoderAbsDeg = absEncoderDeg;
        return absEncoderDeg;
    }

    private double remapTargetIntoSafeRange(double desiredTargetDeg, double referenceDeg) {
        double softMin = RobotConstants.getTurretSoftMinDeg();
        double softMax = RobotConstants.getTurretSoftMaxDeg();

        int kMin = (int) Math.ceil((softMin - desiredTargetDeg) / 360.0);
        int kMax = (int) Math.floor((softMax - desiredTargetDeg) / 360.0);

        if (kMin <= kMax) {
            double bestCandidate = desiredTargetDeg + 360.0 * kMin;
            double bestDistance = Math.abs(bestCandidate - referenceDeg);

            for (int k = kMin + 1; k <= kMax; k++) {
                double candidate = desiredTargetDeg + 360.0 * k;
                double distance = Math.abs(candidate - referenceDeg);
                if (distance < bestDistance) {
                    bestCandidate = candidate;
                    bestDistance = distance;
                }
            }

            return Range.clip(bestCandidate, softMin, softMax);
        }

        return Range.clip(desiredTargetDeg, softMin, softMax);
    }

    private void setTurretPosition(double turretDeg) {
        double adjustedDeg = turretDeg + RobotConstants.ZERO_OFFSET_DEG;

        if (RobotConstants.INVERT) {
            adjustedDeg = RobotConstants.MAX_TURRET_DEG - adjustedDeg;
        }

        adjustedDeg = Range.clip(adjustedDeg, RobotConstants.MIN_TURRET_DEG, RobotConstants.MAX_TURRET_DEG);

        currentCommandedAngle = turretDeg;

        dbg_inputTurretDeg = turretDeg;
        dbg_adjustedDeg = adjustedDeg;
        dbg_servoDeg = adjustedDeg / RobotConstants.GEAR_RATIO;
        dbg_servoPos = Range.clip(dbg_servoDeg / RobotConstants.SERVO_RANGE_DEG, 0.0, 1.0);
    }

    private void setServoPowerCached(double power) {
        if (Double.isNaN(lastPower) || Math.abs(power - lastPower) > RobotConstants.TURRET_SERVO_THRESHOLD) {
            servo1.setPower(power);
            servo2.setPower(power);
            servo3.setPower(power);
            lastPower = power;
        }
        dbg_commandedPower = power;
    }

    public void update(double dx, double dy, double heading){
        update(dx, dy, heading, 0);
    }

    public void update(double dx, double dy, double heading, double angularVelocityRadPerSec){
        update(dx, dy, heading, angularVelocityRadPerSec, 12.0);
    }

    public void update(double dx, double dy, double heading, double angularVelocityRadPerSec, double voltage){
        if(isTracking) {
            double angVelDegPerSec = Math.toDegrees(angularVelocityRadPerSec);
            double leadDeg = angVelDegPerSec * RobotConstants.TURRET_ANG_VEL_LEAD_SEC;
            double robotHeading = Math.toDegrees(heading) + leadDeg;
            double headingToGoal = Math.toDegrees(Math.atan2(dy, dx));
            double rawTarget = headingToGoal - robotHeading + RobotConstants.TURRET_FORWARD_OFFSET_DEG;

            dbg_angVelDegPerSec = angVelDegPerSec;
            dbg_angVelLeadDeg = leadDeg;

            //shortest-path: pick the 360°-equivalent of rawTarget closest to current commanded angle.
            //work in offset-less frame; setAngle re-applies offsetDegrees.
            double currentRawCommanded = currentCommandedAngle - offsetDegrees;
            double diff = rawTarget - currentRawCommanded;
            diff -= 360 * Math.round(diff / 360.0);    //wrap diff to [-180, 180]
            double targetTurretAngle = currentRawCommanded + diff;

            dbg_xToGoal = dx;
            dbg_yToGoal = dy;
            dbg_robotHeadingDeg = robotHeading;
            dbg_headingToGoalDeg = headingToGoal;
            dbg_rawTargetTurretAngle = rawTarget;

            setAngle(targetTurretAngle);
        }

        //keep aiming through wire-range wraparounds by choosing a valid 360°-equivalent target before final clamp.
        targetAngle = remapTargetIntoSafeRange(targetAngle, currentCommandedAngle);
        dbg_clampedTargetTurretAngle = targetAngle - offsetDegrees;

        double commandAngle = targetAngle;
        if (RobotConstants.SLEW_RATE_ENABLED) {
            double elapsedSec = slewTimer.seconds();
            double maxDelta = RobotConstants.SLEW_RATE_DEG_PER_SEC * elapsedSec;
            commandAngle = currentCommandedAngle + Range.clip(targetAngle - currentCommandedAngle, -maxDelta, maxDelta);
        }
        slewTimer.reset();

        //store commanded angle + refresh open-loop dbg fields
        setTurretPosition(commandAngle);

        double encoderDeg = readEncoderUnwrapped();

        double error = Math.abs(commandAngle - encoderDeg);

        double enterClose = RobotConstants.TURRET_PID_SWITCH_THRESHOLD_DEG - RobotConstants.TURRET_PID_SWITCH_HYSTERESIS_DEG;
        double exitClose  = RobotConstants.TURRET_PID_SWITCH_THRESHOLD_DEG + RobotConstants.TURRET_PID_SWITCH_HYSTERESIS_DEG;
        if (useClosePid) {
            if (error > exitClose) useClosePid = false;
        } else {
            if (error < enterClose) useClosePid = true;
        }

        pidClose.setGains(RobotConstants.TURRET_P_CLOSE, useClosePid ? RobotConstants.TURRET_I_CLOSE : 0,
                RobotConstants.TURRET_D_CLOSE, RobotConstants.TURRET_F_CLOSE);
        pidFar.setGains(RobotConstants.TURRET_P_FAR, useClosePid ? 0 : RobotConstants.TURRET_I_FAR,
                RobotConstants.TURRET_D_FAR, RobotConstants.TURRET_F_FAR);

        double closeOut = pidClose.update(commandAngle, encoderDeg, voltage);
        double farOut   = pidFar.update(commandAngle, encoderDeg, voltage);
        double pidOut   = useClosePid ? closeOut : farOut;

        dbg_pidError = commandAngle - encoderDeg;
        dbg_pidOutput = pidOut;

        if (RobotConstants.TURRET_PID_OUTPUT_INVERT) pidOut = -pidOut;
        pidOut = Range.clip(pidOut, -RobotConstants.TURRET_MAX_POWER, RobotConstants.TURRET_MAX_POWER);
        setServoPowerCached(pidOut);
    }

}
