package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal;
import org.firstinspires.ftc.teamcode.ultimategoal.UGoalRobot;

@Config
@Autonomous(name="RRAutoTest", group = "RR")
public class AutoTest extends LinearOpMode {

    public static double DISTANCE = 60; // in
    RRMecanumDrive drive = new RRMecanumDrive(hardwareMap);
    UGoalRobot robot = new UGoalRobot(hardwareMap, this);

    @Override
    public void runOpMode() throws InterruptedException {



        Trajectory goToShoot = drive.trajectoryBuilder(new Pose2d(-62, 32, 0))
                .splineTo(new Vector2d(-34, 20), 0)  // 12 inches right, 28 inches forward
                .splineTo(new Vector2d(-6, 28), 0)  // 8 inches left, another 28 inches forward
                .build();



        Trajectory placeWobble = drive.trajectoryBuilder(goToShoot.end())
                .splineTo(new Vector2d(FieldUGoal.TILE_3_CENTER, FieldUGoal.TILE_2_CENTER + 4), 0)
                //.strafeLeft(FieldUGoal.TILE_LENGTH*0.5)
                .build();





        Trajectory secondWobble = drive.trajectoryBuilder(placeWobble.end(), true)
                .splineTo(new Vector2d(-FieldUGoal.TILE_1_FROM_ORIGIN, FieldUGoal.TILE_2_CENTER + 4), 0)
                .build();



        Trajectory placeSecondWobble = drive.trajectoryBuilder(secondWobble.end())
                .splineTo(new Vector2d(FieldUGoal.TILE_3_CENTER, FieldUGoal.TILE_2_CENTER + 4), 0)
                .build();

        Trajectory park = drive.trajectoryBuilder(placeSecondWobble.end())
                .lineTo(new Vector2d(6, 28))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        robot.runShooterFlywheel();

        drive.followTrajectory(goToShoot);

        shootRingsIntoHighGoal();

        drive.followTrajectory(placeWobble);

        robot.deliverWobble();
        sleep(250);
        robot.setWobbleArmPickup();

        drive.followTrajectory(secondWobble);

        robot.pickUpWobble();

        drive.followTrajectory(placeSecondWobble);

        robot.deliverWobble();
        sleep(250);
        robot.setWobbleArmDown();

        drive.followTrajectory(park);



        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }

    public void shootRingsIntoHighGoal(){
        // assumption that flywheel is already running so it can gain full speed

        // Note: removed a flip4Red method around FieldUGoal.GOALY, add this back in when support for both sides is added
        robot.tiltShooterPlatform(FieldUGoal.GOALX, FieldUGoal.GOALY, FieldUGoal.HIGH_GOAL_HEIGHT);
        // the pusher seems to miss 3rd ring because it hasn't fallen down into the collector yet
        // therefore we run the intake to help 3rd ring drop down and try to shoot 5 times
        robot.runIntake(1.0);
        for (int i = 0; i<5; i++) {
            sleep(1200); // allow some time for the flywheel to gain full speed after each shot
            robot.shootRing();
        }
        robot.stopIntake();
        robot.stopShooterFlywheel();
    }


}
