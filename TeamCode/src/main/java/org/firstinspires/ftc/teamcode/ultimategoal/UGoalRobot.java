package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.MecabotMove;

import java.lang.reflect.Field;

public class UGoalRobot extends MecabotMove {


    public UGoalRobot(HardwareMap ahwMap, LinearOpMode opMode) {
        super(ahwMap, opMode);
        this.init(ahwMap);
    }

    // Motors
    public DcMotor angleMotor = null;
    public DcMotor launcherMotor = null;
    public DcMotor liftMotor = null;

    //Servos
    public Servo launcherServo = null;
    public Servo releaseIntake = null;
    //finger is for wobble pickup, claw is for putting rings on wobble
    public Servo wobbleFinger = null;
    public Servo wobbleClaw = null;
    public Servo wobbleClawArm = null;
    // intake motor and servo
    public DcMotor intakeMotor = null;
    public DcMotor wobbleFingerArm = null;

    //Finals
    static final double ENCODER_TICKS_PER_ROTATION  = 537.6f; // goBilda 5202 series Yellow Jacket Planetary 19.2:1 gear ratio, 312 RPM


    // The hardware map obtained from OpMode
    HardwareMap hwMap;

    // Initialization
    public void init(HardwareMap ahwMap) {

        angleMotor = ahwMap.get(DcMotor.class, "launcherMotor");
        launcherMotor = ahwMap.get(DcMotor.class, "flywheelMotor");
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        wobbleFingerArm = hwMap.get(DcMotor.class, "wobbleFingerArm");
        liftMotor = hwMap.get(DcMotor.class, "liftMotor");

        angleMotor.setDirection(DcMotor.Direction.REVERSE);
        //*TODO is wobbleFingerArm reverse or not to work with positive encoder counts
        //wobbleFingerArm.setDirection(DcMotor.Direction.REVERSE);

        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleFingerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        angleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleFingerArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        releaseIntake = ahwMap.get(Servo.class, "releaseIntake");
        wobbleFinger = ahwMap.get(Servo.class, "wobbleFinger");
        wobbleClaw = ahwMap.get(Servo.class, "wobbleClaw");
        wobbleClawArm = ahwMap.get(Servo.class, "wobbleClawArm");
        launcherServo = ahwMap.get(Servo.class, "launcherServo");

        launcherServo.setPosition(FieldUGoal.PUSHER_REST_POSITION);
        wobbleFinger.setPosition(FieldUGoal.WOBBLE_FINGER_CLOSED);
        wobbleClaw.setPosition(FieldUGoal.WOBBLE_CLAW_OPEN);
        wobbleClawArm.setPosition(FieldUGoal.WOBBLE_CLAW_ARM_INSIDE);

    }



    // This method unfolds the finger arm and grabs the wobble
    // Then it moves the arm to vertical position so it doesn't drag on the ground in auto, or to clear wall in endgame
    // During auto, we want to place the wobble goal touching the robot on the right side so we can immediately grab it
    // Alternative design is for hardware to allow preloading wobble inside robot
    public void pickUpWobble(double speed){//at beginning of auto or for teleop
        //set to run to position for autonomous
        wobbleFingerArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //finger arm motor 0 position will be straight down, folded inside the robot when we put the robot on start line
        //open to get ready to pickup
        wobbleFinger.setPosition(FieldUGoal.WOBBLE_FINGER_OPEN);
        //wobble arm motor needs to unfold 90 degrees to horizontal position
        wobbleFingerArm.setTargetPosition(FieldUGoal.FINGER_ARM_HORIZONTAL);
        wobbleFingerArm.setPower(speed);
        //grab wobble
        wobbleFinger.setPosition(FieldUGoal.WOBBLE_FINGER_CLOSED);
        //bring the wobble arm up 180 degrees all the way up so we don't drag it
        wobbleFingerArm.setTargetPosition(FieldUGoal.FINGER_ARM_UP);
        wobbleFingerArm.setPower(speed);
    }


    //*TODO delivers wobble X inches from the robot center to the right, assuming launcher is the front
    // This assumes we have a wobble goal held and is held vertically
    // It lowers arm to horizontal position and drops the goal
    // Then it folds arm back into robot
    // speed determines how fast the finger arm motor moves
    public void placeWobble(double speed) {
        //set to run to position for autonomous
        wobbleFingerArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //brings wobble arm down to 90 degrees
        wobbleFingerArm.setTargetPosition(FieldUGoal.FINGER_ARM_HORIZONTAL);
        wobbleFingerArm.setPower(speed);
        wobbleFinger.setPosition(FieldUGoal.WOBBLE_FINGER_OPEN);

        //reset and put Finger Arm back into the robot out of the way
        wobbleFingerArm.setTargetPosition(FieldUGoal.FINGER_ARM_DOWN);
        wobbleFingerArm.setPower(speed);
        wobbleFinger.setPosition(FieldUGoal.WOBBLE_FINGER_CLOSED);
    }
    //picks up stored rings in robot to put on wobble goal in the endgame
    public void grabRingsWithClaw(){
        wobbleClaw.setPosition(FieldUGoal.WOBBLE_CLAW_CLOSED);
    }
    //used to lift claw with rings out of robot onto wobble goal
    //*TODO PROGRAM lift
    //positive power is (up/down?)
    //hard stop needs to be programmed to prevent breaking string
    //for auto, set to RUN_TO_POSITION
    public void wobbleLift(){

    }
    //rings will be put on the wobble on the RIGHT side of the robot
    //we will have to turn around to grab it on the LEFT side
    //uses lift to lift claw, swings it to outside, then lift to lower rings onto wobble goal
    //when done, it resets by lowering lift and folding the claw and arm back into robot
    public void putRingsOnWobble(){
        //*TODO use lift
        grabRingsWithClaw();
        wobbleClawArm.setPosition(FieldUGoal.WOBBLE_CLAW_ARM_OUTSIDE);
        wobbleClaw.setPosition(FieldUGoal.WOBBLE_CLAW_OPEN);
        //reset and put claw back into robot out of the way
        wobbleClaw.setPosition(FieldUGoal.WOBBLE_CLAW_CLOSED);
        wobbleClawArm.setPosition(FieldUGoal.WOBBLE_CLAW_ARM_INSIDE);



    }

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

