package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Pure Servo Test", group = "Test Teleops")
public class PureServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo latch = hardwareMap.get(Servo.class, "latchservo");
        latch.setPosition(1);
//        ElapsedTime timer = new ElapsedTime();
//        boolean goingToOne = true;x
//
//        telemetry.addLine("Servo grabbed. Press Start.");
//        telemetry.update();
//
        waitForStart();
//
//        latch.setPosition(0.0);
//        timer.reset();
//
        while (opModeIsActive()) {
            latch.setPosition(1);

//            if (timer.seconds() > 1.0) {
//                if (goingToOne) {
//                    latch.setPosition(1.0);
//                } else {
//                    latch.setPosition(0.0);
//                }
//                goingToOne = !goingToOne;
//                timer.reset();
//            }
//
//            telemetry.addData("Target", goingToOne ? "1.0 next" : "0.0 next");
//            telemetry.addData("Timer", "%.2f", timer.seconds());
//            telemetry.update();
        }
    }
}
