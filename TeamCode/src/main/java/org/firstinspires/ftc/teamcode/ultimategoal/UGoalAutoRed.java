package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.AllianceColor;


@Autonomous(name="RED Full Auto", group="QT")

public class UGoalAutoRed extends UGoalAutoBase {

    @Override
    public String getColorString(){
        return "RED";
    }

    @Override
    public void setOdometryStartingPosition() {

        // Robot is 17 x 17 square, robot position (x,y) is center of the robot
        // Starting X on middle of starting line. Starting line is on the third tile, middle of start line is center of 3rd tile
        // Starting Y is on inner start line, which is one tile from origin. Red is Negative, Q3
        // Launcher facing toward center of field is 0 degrees
        globalPosition.initGlobalPosition(FieldUGoal.TILE_3_CENTER, -FieldUGoal.TILE_1_FROM_ORIGIN, 0.0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

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
