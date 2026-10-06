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
@TeleOp(name = "Film Teleop", group = "Test Teleops")
public class FilmTeleop extends LinearOpMode {

    private Follower follower;
    private static Pose startPose;

    private Robot r;
    private static int alliance = 1;

    private boolean intakeOn = false;
    private boolean intakeReverse = false;
    boolean wasStalled = false;
    private static boolean teleop = false;

    private double intake = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //initlizating
        //startPose = new Pose(120.86, 120.42, Math.toRadians(40.19));
        startPose = new Pose(46.5, 128, Math.toRadians(180));
        r = new Robot(hardwareMap, Alliance.BLUE, Teleop.FALSE, startPose);


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
                    intake = 0;
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
                    intake = 0;
                }
            }


            if(gamepad1.dpadLeftWasPressed()){
                intake += 0.1;
                r.intake.setPower(intake);
            }

            if(gamepad1.dpadUpWasPressed()){
                r.shooter.setHood(0);
            }
            if(gamepad1.dpadDownWasPressed()){
                r.shooter.setHood(1);
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



            r.update(follower.getPose(), follower.getVelocity());

            telemetry.addData("shooter RPM", r.shooter.getCurrentRPM());
            // telemetry.addData("limelight distance", r.getLLDistance());
            telemetry.addData("turret angle", r.turret.getCurrentAngle());
            telemetry.addData("turret target angle", r.turret.getTargetAngle());
            telemetry.addData("robot x", follower.getPose().getX());
            telemetry.addData("robot Y", follower.getPose().getY());
            telemetry.addData("robot heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("fusion x", r.getFusionPose().getX());
            telemetry.addData("fusion y", r.getFusionPose().getY());
            telemetry.update();


        }

    }
}