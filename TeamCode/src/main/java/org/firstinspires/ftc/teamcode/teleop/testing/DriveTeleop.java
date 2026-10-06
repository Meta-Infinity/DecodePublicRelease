package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainHardware;

@Config
@TeleOp(name = "Drive Only" , group = "Testing")
public class DriveTeleop extends LinearOpMode {

    private DrivetrainHardware drivetrain;

    private double slowSpeed = 0.6;
    private double fastSpeed = 1.0;
    private static double SHOOTERRPM = 0;


    private double currentSpeed = fastSpeed;
    @Override
    public void runOpMode() throws InterruptedException {
        //initlizating

        drivetrain = new DrivetrainHardware(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {


            //speed shifter
            if (gamepad1.b) {
                currentSpeed = slowSpeed;
            } else if (gamepad1.a) {
                currentSpeed = fastSpeed;
            }



            //driving
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            drivetrain.drive(y, x, rx, currentSpeed);


            telemetry.update();




        }

    }
}