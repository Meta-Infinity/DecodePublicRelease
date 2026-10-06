package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RelocalizePosition;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.StartPosition;
import org.firstinspires.ftc.teamcode.util.Teleop;

@TeleOp(name = "Interp Testing (Red)", group = "Testing")
public class InterpTestingOpMode extends RobotTeleop {

    @Override
    protected RobotConfig getConfig() {
        return new RobotConfig(StartPosition.RED, Alliance.RED, Teleop.TRUE, RelocalizePosition.RED_CORNER);
    }

    @Override
    protected void shooter() {
        // Manual override from Dashboard constants
        r.shooter.setRPM(RobotConstants.TEST_SHOOTER_RPM);
        r.shooter.setHood(RobotConstants.TEST_HOOD_POSITION);
        
        telemetry.addData("TESTING MODE", "MANUAL SHOOTER OVERRIDE");
        telemetry.addData("Manual Target RPM", RobotConstants.TEST_SHOOTER_RPM);
        telemetry.addData("Manual Target Hood", RobotConstants.TEST_HOOD_POSITION);
        telemetry.addData("Distance to Goal", r.shooter.getDistance());
    }
}
