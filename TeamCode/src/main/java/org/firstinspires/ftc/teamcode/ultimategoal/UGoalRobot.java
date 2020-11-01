package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.MecabotMove;

public class UGoalRobot extends MecabotMove {

    // TARGETS
    public static final double HIGH_GOAL = 48;
    public static final double MED_GOAL = 36;
    public static final double LOW_GOAL = 24;

    public UGoalRobot(HardwareMap ahwMap, LinearOpMode opMode) {
        super(ahwMap, opMode);
        this.init(ahwMap);
    }

    // Motors
    public DcMotor launcherMotor = null;


    //Finals
    public final double GOAL_HEIGHT = 48;
    static final double ENCODER_TICKS_PER_ROTATION  = 537.6f; // goBilda 5202 series Yellow Jacket Planetary 19.2:1 gear ratio, 312 RPM


    // The hardware map obtained from OpMode
    HardwareMap hwMap;

    // Initialization
    public void init(HardwareMap ahwMap) {

        launcherMotor = ahwMap.get(DcMotor.class, "launcherMotor");
        launcherMotor.setDirection(DcMotor.Direction.REVERSE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }




    // This assumes shooting at red goal
    // This method gives the horizontal distance to goal, in other words the distance on the ground
    public double distanceToShoot(double x, double y) {
        double robotX = x;
        double robotY = y;

        double targetX = 72;
        double targetY = -36;

        double xDiff = targetX - robotX;
        double yDiff = targetY - robotY;

        return Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
    }

    // This assumes shooting at red goal
    // This method gives the angel from the robot to the goal
    public double robotAngleToShoot(double x, double y) {
        double robotX = x;
        double robotY = y;

        double targetX = 72;
        double targetY = -36;

        double xDiff = targetX - robotX;
        double yDiff = targetY - robotY;


        return Math.atan2(xDiff, yDiff);
    }

    public double launcherAngleToShoot(double x, double y) {
        double xDist = distanceToShoot(x, y); //Distance on the ground
        double yDist = GOAL_HEIGHT; // HEIGHT of the goal

        return Math.atan(yDist/xDist);
    }

    public void shoot(double target) {

        odometryRotateToHeading(robotAngleToShoot(globalPosition.getXinches(), globalPosition.getYinches()), 0.5, 5, true);

        double launcherAngle = launcherAngleToShoot(globalPosition.getXinches(), globalPosition.getYinches());

        // MATH to covert angle to rotation of oval things under launcher
        double ovalRotation = 0;

        // Convertion of rotation to encoder ticks
        double ovalRotationTicks = ENCODER_TICKS_PER_ROTATION / ovalRotation;

        // Encoder movement for launcher motor (not drivetrain encoder methods)
        launcherMotor.setTargetPosition((int) ovalRotationTicks);
        launcherMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}