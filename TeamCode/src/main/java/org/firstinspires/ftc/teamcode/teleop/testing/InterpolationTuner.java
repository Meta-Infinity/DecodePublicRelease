package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.RobotTeleop;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RelocalizePosition;
import org.firstinspires.ftc.teamcode.util.StartPosition;
import org.firstinspires.ftc.teamcode.util.Teleop;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

@Config
@TeleOp(name = "Interpolation Tuner", group = "Test Teleops")
public class InterpolationTuner extends RobotTeleop {

    public static double TUNE_HOOD = 0.30;
    public static double TUNE_RPM = 3200;
    public static boolean SAVE = false;
    public static boolean DELETE_LAST = false;
    public static boolean CLEAR_ALL = false;
    public static double DEDUPE_TOL_IN = 1.5;

    private final List<double[]> samples = new ArrayList<>();

    @Override
    protected RobotConfig getConfig() {
        return new RobotConfig(StartPosition.RED, Alliance.RED, Teleop.TRUE, RelocalizePosition.RED_CORNER);
    }

    @Override
    protected void update() {
        r.shooter.setRPM(TUNE_RPM);
        r.shooter.setHood(TUNE_HOOD);
        super.update();
    }

    @Override
    protected void shooter() {
        if (SAVE) {
            SAVE = false;
            saveSample();
        }
        if (DELETE_LAST) {
            DELETE_LAST = false;
            if (!samples.isEmpty()) samples.remove(samples.size() - 1);
        }
        if (CLEAR_ALL) {
            CLEAR_ALL = false;
            samples.clear();
        }
    }

    private void saveSample() {
        double d = r.shooter.getDistance();
        for (int i = 0; i < samples.size(); i++) {
            if (Math.abs(samples.get(i)[0] - d) < DEDUPE_TOL_IN) {
                samples.set(i, new double[]{d, TUNE_HOOD, TUNE_RPM});
                return;
            }
        }
        samples.add(new double[]{d, TUNE_HOOD, TUNE_RPM});
    }

    @Override
    protected void telemetry(double loopTime) {
        telemetry.addData("Distance (in)", r.round(r.shooter.getDistance()));
        telemetry.addData("Typed Hood", TUNE_HOOD);
        telemetry.addData("Actual Hood", r.round(r.shooter.getHoodPosition()));
        telemetry.addData("Typed RPM", TUNE_RPM);
        telemetry.addData("Actual RPM", r.round(r.shooter.getCurrentRPM()));
        telemetry.addData("Samples", samples.size());
        telemetry.addData("Loop ms", r.round(loopTime));

        List<double[]> sorted = new ArrayList<>(samples);
        Collections.sort(sorted, new Comparator<double[]>() {
            @Override
            public int compare(double[] a, double[] b) {
                return Double.compare(a[0], b[0]);
            }
        });

        telemetry.addLine("--- paste into LookUpTable.java ---");
        for (double[] s : sorted) {
            telemetry.addLine(String.format(Locale.US,
                    "rpm.add(%.1f, %d); hood.add(%.1f, %.3f);",
                    s[0], (int) Math.round(s[2]), s[0], s[1]));
        }

        telemetry.update();
    }
}
