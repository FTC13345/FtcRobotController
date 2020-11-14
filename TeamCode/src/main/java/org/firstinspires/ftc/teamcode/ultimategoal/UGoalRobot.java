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
    //constants
    static final double     INTAKE_DOWN_ANGLE           = Servo.MAX_POSITION; //max is 135 degrees, all the way down
    static final double     PUSHER_REST_POSITION        = Servo.MIN_POSITION;
    static final double     WOBBLE_FINGER_CLOSED        = Servo.MIN_POSITION;
    static final double     WOBBLE_FINGER_OPEN          = 0.5; //middle to save time
    static final double     WOBBLE_CLAW_OPEN            = Servo.MAX_POSITION;
    static final double     WOBBLE_CLAW_CLOSED          = 0;//CHANGE WITH TESTING
    static final double     WOBBLE_CLAW_ARM_INSIDE      = Servo.MIN_POSITION;//needs to be 180 degrees
    static final double     WOBBLE_CLAW_ARM_OUTSIDE     = 0;// 0 degrees
    static final int        ENCODER_TICKS_PER_REVOLUTION        = 288;

    static final int        FINGER_ARM_HORIZONTAL       = ENCODER_TICKS_PER_REVOLUTION/2;
    static final int        FINGER_ARM_UP               = ENCODER_TICKS_PER_REVOLUTION/4;
    static final int        FINGER_ARM_DOWN             = 0;

    //*TODO FIND LIFT TOP AND BOTTOM VALUES, AND TEST FOR WOBBLE RINGS DISTANCE
    static final int        LIFT_TOP                    = 0;
    static final int        LIFT_BOTTOM                 = 0;
    static final int        LIFT_UP_RINGS_HEIGHT        = 0;

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
        //reverse with current hardware, but we can just reverse the string on the spool easily to work with positive
        liftMotor.setDirection(DcMotor.Direction.REVERSE);

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

        launcherServo.setPosition(PUSHER_REST_POSITION);
        wobbleFinger.setPosition(WOBBLE_FINGER_CLOSED);
        wobbleClaw.setPosition(WOBBLE_CLAW_OPEN);
        wobbleClawArm.setPosition(WOBBLE_CLAW_ARM_INSIDE);

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
        wobbleFinger.setPosition(WOBBLE_FINGER_OPEN);
        //wobble arm motor needs to unfold 90 degrees to horizontal position
        wobbleFingerArm.setTargetPosition(FINGER_ARM_HORIZONTAL);
        wobbleFingerArm.setPower(speed);
        //grab wobble
        wobbleFinger.setPosition(WOBBLE_FINGER_CLOSED);
        //bring the wobble arm up 180 degrees all the way up so we don't drag it
        wobbleFingerArm.setTargetPosition(FINGER_ARM_UP);
        wobbleFingerArm.setPower(speed);
    }


    // This assumes we have a wobble goal held and is held vertically
    // It lowers arm to horizontal position and drops the goal
    // Then it folds arm back into robot
    // speed determines how fast the finger arm motor moves
    public void placeWobble(double speed) {
        //set to run to position for autonomous
        wobbleFingerArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //brings wobble arm down to 90 degrees
        wobbleFingerArm.setTargetPosition(FINGER_ARM_HORIZONTAL);
        wobbleFingerArm.setPower(speed);
        wobbleFinger.setPosition(WOBBLE_FINGER_OPEN);

        //reset and put Finger Arm back into the robot out of the way
        wobbleFingerArm.setTargetPosition(FINGER_ARM_DOWN);
        wobbleFingerArm.setPower(speed);
        wobbleFinger.setPosition(WOBBLE_FINGER_CLOSED);
    }
    //picks up stored rings in robot to put on wobble goal in the endgame
    public void grabRingsWithClaw(){
        wobbleClaw.setPosition(WOBBLE_CLAW_CLOSED);
    }
    //used to lift claw with rings out of robot onto wobble goal
    //positive power is (up/down?)
    //hard stop needs to be programmed to prevent breaking string
    //for auto, set to RUN_TO_POSITION

    /**
     * This is the method for AUTONOMOUS
     * liftMotor is set to run to position, and uses the parameter to determine that distance
     * constant LIFT_UP_RINGS_HEIGHT is the height we lift the claw to deposit rings onto wobble goal
     * @param height is encoder counts that tells the lift how high to go
     */
    public void wobbleLift(int height){
        //*TODO test for stops in teleop first!
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(height);
        liftMotor.setPower(MecabotMove.DRIVE_SPEED_DEFAULT);


    }
    //rings will be put on the wobble on the RIGHT side of the robot
    //we will have to turn around to grab it on the LEFT side
    //uses lift to lift claw, swings it to outside, then lift to lower rings onto wobble goal
    //when done, it resets by lowering lift and folding the claw and arm back into robot

    public void putRingsOnWobble(){
        //pickup rings inside robot
        grabRingsWithClaw();
        //lift the claw to predetermined height
        //This gives room to swing out claw and drop the rings onto wobble
        wobbleLift(LIFT_UP_RINGS_HEIGHT);
        //rotates claw outside to put it outside the robot next to wobble goal
        wobbleClawArm.setPosition(WOBBLE_CLAW_ARM_OUTSIDE);
        wobbleClaw.setPosition(WOBBLE_CLAW_OPEN);
        //reset, lower lift and put claw back into robot out of the way
        wobbleLift(LIFT_BOTTOM);
        wobbleClaw.setPosition(WOBBLE_CLAW_CLOSED);
        wobbleClawArm.setPosition(WOBBLE_CLAW_ARM_INSIDE);
    }

    public void stopLift(){
        //stop and brake lift
        intakeMotor.setPower(0);
    }

    public double distanceToShoot(double robotX, double robotY) {

        double targetX = 72;
        double targetY = -36;

        double xDiff = targetX - robotX;
        double yDiff = targetY - robotY;

        return Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
    }
    //only for autonomous, for teleop use function that doens't call this
    // This assumes blue, for red use flip4Red when calling
    // This method gives the angel from the robot to the goal
    public double robotAngleToShoot(double x, double y, int targetX, int targetY) {
        double robotX = x;
        double robotY = y;


        double xDiff = targetX - robotX;
        double yDiff = targetY - robotY;


        return Math.atan2(xDiff, yDiff);
    }
    //returns angle that launch platform needs to be tilted to aim at the target
    public double launcherAngleToShoot(double x, double y, double targetHeight) {
        double xDist = distanceToShoot(x, y); //Distance on the ground
        double yDist = targetHeight; // HEIGHT of the goal

        return Math.atan(yDist/xDist);
    }

    public void tiltLaunchPlatform(double targetX, double targetY, double targetHeight) {

        odometryRotateToHeading(robotAngleToShoot(globalPosition.getXinches(), globalPosition.getYinches(), targetX, targetY), 0.5, 5, true);

        double launcherAngle = launcherAngleToShoot(globalPosition.getXinches(), globalPosition.getYinches(), targetHeight);

        // MATH to covert angle to rotation of oval things under launcher
        double ovalRotation = (launcherAngle*FieldUGoal.CONVERT_RADIANS_TO_DEGREES - 20) * 7.2; // 7.2 scales 25 to 180 (range is 20-45 transformed to 0-180)

        // Convertion of rotation to encoder ticks
        double ovalRotationTicks = ENCODER_TICKS_PER_ROTATION / ovalRotation;

        // Encoder movement for launcher motor (not drivetrain encoder methods)
        angleMotor.setTargetPosition((int) ovalRotationTicks);
        angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    // If we don't want to turn the robot, in TeleOp usually, don't pass the target x and y
    // We can't use this in teleop because we can't differentiate red and blue y coords, and there aren't enough buttons for each powershot.
    public void tiltLaunchPlatform(double targetHeight) {

        double launcherAngle = launcherAngleToShoot(globalPosition.getXinches(), globalPosition.getYinches(), targetHeight);

        // MATH to covert angle to rotation of oval things under launcher
        double ovalRotation = (launcherAngle*FieldUGoal.CONVERT_RADIANS_TO_DEGREES - 20) * 7.2; // 7.2 scales 25 to 180 (range is 20-45 transformed to 0-180)

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
        releaseIntake.setPosition(INTAKE_DOWN_ANGLE);
    }
    public void runIntake(){
        intakeMotor.setPower(1);
    }
    public void stopIntake(){
        intakeMotor.setPower(0);
    }

    public void runIntake(double power) {
        intakeMotor.setPower(power);
    }

}

