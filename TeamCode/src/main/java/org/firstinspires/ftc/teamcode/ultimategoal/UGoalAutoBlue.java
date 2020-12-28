package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.AllianceColor;


@Autonomous(name="BLUE Full Auto", group="QT")

public class UGoalAutoBlue extends UGoalAutoBase {

    @Override
    public String getColorString(){
        return "BLUE";
    }

    @Override
    public void setOdometryStartingPosition() {

        // Robot is 17 x 17 square, robot position (x,y) is center of the robot
        // Starting X on middle of starting line. Starting line is on the third tile, middle of start line is center of 3rd tile
        // Starting Y is on inner start line, which is one tile from origin. Blue is positive, Q2
        // Launcher facing toward center of field is 0 degrees
        globalPosition.initGlobalPosition(-FieldUGoal.TILE_3_FROM_ORIGIN+FieldUGoal.ROBOT_RADIUS, FieldUGoal.TILE_1_FROM_ORIGIN+FieldUGoal.ROBOT_RADIUS, 0.0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        FieldUGoal.aColor = AllianceColor.BLUE;

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
