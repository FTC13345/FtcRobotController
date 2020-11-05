package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.MecabotMove;

public class UGoalRobot extends MecabotMove {


    public UGoalRobot(HardwareMap ahwMap, LinearOpMode opMode) {
        super(ahwMap, opMode);
        this.init(ahwMap);
    }

    // Motors
    public DcMotor angleMotor = null;
    public DcMotor launcherMotor = null;

    //Servos
    public Servo launcherServo = null;
    public Servo releaseIntake = null;
    // intake motor and servo
    public DcMotor intakeMotor = null;

    //Finals
    static final double ENCODER_TICKS_PER_ROTATION  = 537.6f; // goBilda 5202 series Yellow Jacket Planetary 19.2:1 gear ratio, 312 RPM


    // The hardware map obtained from OpMode
    HardwareMap hwMap;

    // Initialization
    public void init(HardwareMap ahwMap) {

        angleMotor = ahwMap.get(DcMotor.class, "launcherMotor");
        launcherMotor = ahwMap.get(DcMotor.class, "flywheelMotor");
        angleMotor.setDirection(DcMotor.Direction.REVERSE);
        angleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        releaseIntake = ahwMap.get(Servo.class, "releaseIntake");

        launcherServo = ahwMap.get(Servo.class, "launcherServo");
        launcherServo.setPosition(FieldUGoal.PUSHER_REST_POSITION);
    }




    // This assumes shooting at red goal
    // This method gives the horizontal distance to goal, in other words the distance on the ground
    public double distanceToShoot(double robotX, double robotY) {

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

    public double launcherAngleToShoot(double x, double y, double targetHeight) {
        double xDist = distanceToShoot(x, y); //Distance on the ground
        double yDist = targetHeight; // HEIGHT of the goal

        return Math.atan(yDist/xDist);
    }

    public void tiltLaunchPlatform(double targetHeight) {

        odometryRotateToHeading(robotAngleToShoot(globalPosition.getXinches(), globalPosition.getYinches()), 0.5, 5, true);

        double launcherAngle = launcherAngleToShoot(globalPosition.getXinches(), globalPosition.getYinches(), targetHeight);

        // MATH to covert angle to rotation of oval things under launcher
        double ovalRotation = (launcherAngle - 20) * 7.2; // 7.2 scales 25 to 180 (range is 20-45 transformed to 0-180)

        // Convertion of rotation to encoder ticks
        double ovalRotationTicks = ENCODER_TICKS_PER_ROTATION / ovalRotation;

        // Encoder movement for launcher motor (not drivetrain encoder methods)
        angleMotor.setTargetPosition((int) ovalRotationTicks);
        angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runLauncherMotor() {
        launcherMotor.setPower(1);
    }

    public void stopLauncherMotor() {
        launcherMotor.setPower(0);
    }

    public void shootRing() {
        launcherServo.setPosition(Servo.MIN_POSITION);
        launcherServo.setPosition(Servo.MAX_POSITION);
    }

    public void releaseIntake(){
        releaseIntake.setPosition(FieldUGoal.INTAKE_DOWN_ANGLE);
    }
    public void runIntake(){
        intakeMotor.setPower(1);
    }
    public void stopIntake(){
        intakeMotor.setPower(0);
    }

}