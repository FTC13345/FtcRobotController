package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal;

@Config
@Autonomous(name="RRAutoTest", group = "RR")
public class AutoTest extends LinearOpMode {

    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        RRMecanumDrive drive = new RRMecanumDrive(hardwareMap);

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

        drive.followTrajectory(goToShoot);

        sleep(4000);

        drive.followTrajectory(placeWobble);

        sleep(2000);

        drive.followTrajectory(secondWobble);

        sleep(2000);

        drive.followTrajectory(placeSecondWobble);

        drive.followTrajectory(park);



        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
