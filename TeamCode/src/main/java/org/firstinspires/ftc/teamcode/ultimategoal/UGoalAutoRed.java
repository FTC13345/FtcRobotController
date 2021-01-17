package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.AllianceColor;

import static org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.ANGLE_POS_X_AXIS;
import static org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.ROBOT_RADIUS;
import static org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.TILE_1_FROM_ORIGIN;
import static org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.TILE_3_FROM_ORIGIN;
import static org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.poseStartH;
import static org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.poseStartX;
import static org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.poseStartY;


@Autonomous(name="RED Full Auto", group="QT")

public class UGoalAutoRed extends UGoalAutoBase {

    @Override
    public String getColorString(){
        return "RED";
    }

    @Override
    public void setPoseStart() {

        // This code assumes Robot starts at a position as follows:
        // Align the left side of the robot with the INSIDE start line (TILE_1_FROM_ORIGIN in Y axis)
        // Robot Heading is pointing to +ve X-axis  (Ring Shooter Platform is facing the goals)
        // Robot back is touching the perimeter wall.
        globalPosition.setGlobalPosition(poseStartX, poseStartY, poseStartH);
        rrmdrive.setPoseEstimate(new Pose2d(poseStartX, poseStartY, poseStartH));
    }

    @Override
    public void runOpMode() {

        FieldUGoal.aColor = AllianceColor.RED;

        // initialize the robot hardware, navigation, IMU, Odometry and Telemetry display
        initializeOpMode();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // This OpMode does all the tasks in the autonomous period of 30 seconds
        runFullAutoProgram();

        // Don't exit, wait for user (driver presses STOP)
        waitForStop();
    }


}
