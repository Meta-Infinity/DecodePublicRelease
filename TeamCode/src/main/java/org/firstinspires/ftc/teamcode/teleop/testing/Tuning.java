package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Teleop;

@Config
@TeleOp(name = "Tuning Teleop", group = "Test Teleops")
public class Tuning extends LinearOpMode {

    private Follower follower;
    private static Pose startPose;

    private Robot r;

    private static int alliance = 0;

    private boolean intakeOn = false;
    private boolean intakeReverse = false;
    boolean wasStalled = false;
    private static boolean teleop = false;

    private static double hood = 0;
    private static double rpm = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //initlizating
        //startPose = new Pose(120.86, 120.42, Math.toRadians(40.19));
        startPose = new Pose(70.5, 70.5, Math.toRadians(90));
        r = new Robot(hardwareMap, Alliance.RED, Teleop.FALSE, startPose);


        //pedro stuff
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();

        waitForStart();
        while(opModeIsActive()) {
            follower.update();



            //driving
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            r.drivetrain.drive(y, x, rx, 1);


            //intake
            if (gamepad1.leftBumperWasPressed()) {
                if (!intakeOn) {
                    intakeOn = true;
                    intakeReverse = false;
                    r.intake.run();
                } else {
                    intakeOn = false;
                    r.intake.stop();
                }
            }

            if (gamepad1.aWasPressed()) {
                if (!intakeReverse) {
                    intakeReverse = true;
                    intakeOn = false;
                    r.intake.reverse();
                } else {
                    intakeReverse = false;
                    r.intake.stop();
                }
            }


            if(r.intake.getIntakeFull() && !wasStalled){
                gamepad2.rumble(500);
                gamepad1.rumble(500);
                wasStalled = r.intake.getIntakeFull();
            }
            if(wasStalled && !r.intake.getIntakeFull()){
                wasStalled = false;
            }





            //transfer
            if (gamepad1.right_bumper) {
                r.shooter.isShooting();
                r.intake.transferShoot();
                r.intake.run();
            } else {
                r.shooter.notShooting();
                r.intake.transferBlock();
            }



            //shooter
            if (gamepad2.y) {
                r.shooter.startTrack();
            }
            if (gamepad2.dpad_up) {
                r.shooter.stopTrack();
            }

            if(gamepad2.b){
                r.turret.startTrack();
            }
            if(gamepad2.dpad_left){
                r.turret.stopTrack();
                r.turret.resetTurret();
            }

            if (gamepad2.a) {
                r.sotmOn();
            }
            if (gamepad2.dpad_down) {
                r.sotmOff();
            }

            if(gamepad2.leftBumperWasPressed()){
                r.turret.addOffSetDegrees(5);
            }
            if(gamepad2.rightBumperWasPressed()){
                r.turret.addOffSetDegrees(-5);
            }



            r.shooter.setHood(hood);
            r.shooter.setRPM(rpm);

            r.update(follower.getPose(), follower.getVelocity());

            telemetry.addData("shooter RPM", r.shooter.getCurrentRPM());
            // telemetry.addData("limelight distance", r.getLLDistance());
            telemetry.addData("real distance", r.shooter.getDistance());
            telemetry.addData("turret", " current: " + r.round(r.turret.getCurrentAngle()) + " target: " + r.round(r.turret.getTargetAngle()));
            telemetry.addData("robot pose", " x: " + r.round(follower.getPose().getX()) + " y: " + r.round(follower.getPose().getY()) + " h: " + r.round(follower.getPose().getHeading()));
            telemetry.addData("fusion pose", " x: " + r.round(r.getFusionPose().getX()) + " y: " + r.round(r.getFusionPose().getY()));
            telemetry.addData("sotm status", r.getSOTM());
            telemetry.addData("track pose", " x: " + r.round(r.getTrackPose().getX()) + " y: " + r.round(r.getTrackPose().getY()));
            telemetry.addData("velocity", " x: " + r.round(follower.getVelocity().getXComponent()) + " y: " + r.round(follower.getVelocity().getYComponent()));
            telemetry.update();

        }

    }
}