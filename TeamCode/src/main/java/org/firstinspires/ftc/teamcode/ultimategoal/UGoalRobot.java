package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.MecabotMove;

public class UGoalRobot extends MecabotMove {


    public UGoalRobot(HardwareMap ahwMap, LinearOpMode opMode) {
        super(ahwMap, opMode);
        this.init(ahwMap);
    }
    // Tuning Tuning: Compensation for robot behavior, it shoots curved to the left, by few inches
    // Perform calculations as if the Robot center was to the left by few inches and shooting hits target straight ahead
    // Given that the Robot is directly facing the goal line (Heading = 0 (+ve X-axis)), we will also
    // actually position on the field to the right of the intended Target Y coordinate
    static final double     ROBOT_SHOOTING_CURVE_OFFSET = 5.5; // inches
    // Tuning Tuning: Compensation for robot behavior, the platform tilt calculations need few degrees uplift
    // It may be due to gravity or physical platform tilt does not match mechanical design assumption
    static final double     ROBOT_PLATFORM_TILT_OFFSET  = 8.0;  // degrees
    //constants
    static final double     INTAKE_DOWN_ANGLE           = Servo.MAX_POSITION; //max is 135 degrees, all the way down
    static final double     RING_PUSHER_IDLE_POSITION   = Servo.MAX_POSITION;
    static final double     RING_PUSHER_SHOOT_POSITION  = Servo.MIN_POSITION;
    static final double     WOBBLE_FINGER_CLOSED        = Servo.MIN_POSITION;
    static final double     WOBBLE_FINGER_OPEN          = 0.5; //middle to save time
    static final double     LIFT_CLAW_OPEN              = Servo.MAX_POSITION;
    static final double     LIFT_CLAW_CLOSED            = Servo.MIN_POSITION;
    static final double     LIFT_ARM_INSIDE             = Servo.MIN_POSITION;
    static final double     LIFT_ARM_OUTSIDE            = Servo.MAX_POSITION;

    static final int        WOBBLE_ARM_TICKS_PER_REVOLUTION = 288;
    static final int        WOBBLE_ARM_HORIZONTAL       = WOBBLE_ARM_TICKS_PER_REVOLUTION /4;
    static final int        WOBBLE_ARM_UP               = WOBBLE_ARM_TICKS_PER_REVOLUTION /2;
    static final int        WOBBLE_ARM_DOWN             = 0;

    static final int        LIFT_TOP                    = 320;
    static final int        LIFT_BOTTOM                 = 10;
    static final int        LIFT_UP_RINGS_HEIGHT        = LIFT_TOP;

    //Finals
    static final double SHOOTER_FLYWHEEL_RUN = 1.0;
    static final double SHOOTER_FLYWHEEL_STOP = 0.0;
    static final double SHOOTER_OVAL_TICKS_PER_ROTATION = 2 * 1425.2f;  // goBilda 5202 series Yellow Jacket Planetary 50.9:1 gear ratio, 117 RPM motor, times 2:1 bevel gear ratio
    static final double SHOOTER_TILT_ANGLE_MIN = 20.0f;
    static final double SHOOTER_TILT_ANGLE_MAX = 45.0f;

    // status variables
    double shooterTiltAngleDesired = SHOOTER_TILT_ANGLE_MIN;
    double shooterTiltAngleClipped = SHOOTER_TILT_ANGLE_MIN;
    // MATH to convert platform tilt angle to rotation of oval supports under platform
    // scales 25 degrees (range is 20-45 degrees) of shooter platform tilt to 180 degrees of oval rotation
    double ovalRotationDegrees = 0.0f;
    // Convertion of oval rotation (degrees) to motor encoder ticks
    int ovalRotationTicks = 0;

    // Motors
    public DcMotor angleMotor = null;
    public DcMotor liftMotor = null;
    public DcMotor intakeMotor = null;
    public DcMotor wobblePickupArm = null;
    // THis is a motor driven by Spark Mini controller which takes Servo PWM input
    // Please see REV Robotics documentation about Spark Mini and example code ConceptRevSPARKMini
    public DcMotorSimple flyWheelMotor = null;

    //Servos
    public Servo ringPusher = null;
    public Servo releaseIntake = null;
    public CRServo intakeServo = null;
    //finger is for wobble pickup, claw is for putting rings on wobble
    public Servo wobbleFinger = null;
    public Servo liftClaw = null;
    public Servo liftArm = null;

    // Initialization
    public void init(HardwareMap ahwMap) {

        angleMotor = ahwMap.get(DcMotor.class, "angleMotor");
        intakeMotor = ahwMap.get(DcMotor.class, "intakeMotor");
        wobblePickupArm = ahwMap.get(DcMotor.class, "wobbleFingerArm");
        liftMotor = ahwMap.get(DcMotor.class, "liftMotor");
        flyWheelMotor = ahwMap.get(DcMotorSimple.class, "launcherMotorSparkMini");

        // direction depends on hardware installation
        angleMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        wobblePickupArm.setDirection(DcMotor.Direction.REVERSE);
        flyWheelMotor.setDirection(DcMotor.Direction.REVERSE);

        angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobblePickupArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        angleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobblePickupArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        releaseIntake = ahwMap.get(Servo.class, "releaseIntake");
        wobbleFinger = ahwMap.get(Servo.class, "wobbleFinger");
        liftClaw = ahwMap.get(Servo.class, "wobbleClaw");
        liftArm = ahwMap.get(Servo.class, "wobbleClawArm");
        ringPusher = ahwMap.get(Servo.class, "launcherServo");
        intakeServo = ahwMap.get(CRServo.class, "intakeServo");

        // releaseIntake should NOT be initialized to any specific position.
        // we want to be able to initialize robot regardless whether intake is lifted up or let down
        wobbleFinger.setPosition(WOBBLE_FINGER_CLOSED);
        liftClaw.setPosition(LIFT_CLAW_OPEN);
        liftArm.setPosition(LIFT_ARM_INSIDE);
        ringPusher.setPosition(RING_PUSHER_IDLE_POSITION);
        intakeServo.setDirection(CRServo.Direction.REVERSE);
    }

    /*
     * Ring Intake methods
     */
    public void releaseIntake(){
        releaseIntake.setPosition(INTAKE_DOWN_ANGLE);
    }

    public void runIntake(double power) {
        intakeMotor.setPower(power);
        intakeServo.setPower(power);
    }

    public void stopIntake(){
        intakeServo.setPower(0);
        intakeMotor.setPower(0);
    }

    /**
     * Ring Shooter Flywheel and Ring Pusher Arm methods
     */
    public void runShooterFlywheel() {
        flyWheelMotor.setPower(SHOOTER_FLYWHEEL_RUN);
    }

    public void stopShooterFlywheel() {
        flyWheelMotor.setPower(SHOOTER_FLYWHEEL_STOP);
    }

    public boolean isShooterFlywheelRunning() {
        return (flyWheelMotor.getPower() != SHOOTER_FLYWHEEL_STOP);
    }

    public void shootRing() {
        ringPusher.setPosition(RING_PUSHER_SHOOT_POSITION);
        myOpMode.sleep(500);
        ringPusher.setPosition(RING_PUSHER_IDLE_POSITION);
    }

    public void loadRing() {
        ringPusher.setPosition(RING_PUSHER_IDLE_POSITION);
    }

    /*
     * Lift, arm and claw operation methods
     */
    public int getLiftCurrentPosition() {
        return liftMotor.getCurrentPosition();
    }

    public void stopLift() {
        //stop and brake lift
        liftMotor.setPower(MOTOR_STOP_SPEED);
        // Get out of the RunMode RUN_TO_POSITION, so manual player control is possible
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void moveLift(int position) {
        position = Range.clip(position, LIFT_BOTTOM, LIFT_TOP);
        liftMotor.setTargetPosition(position);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(DRIVE_SPEED_DEFAULT);
    }

    public void rotateClawInside() {
        liftArm.setPosition(LIFT_ARM_INSIDE);
    }
    public void rotateClawOutside() {
        liftArm.setPosition(LIFT_ARM_OUTSIDE);
    }
    public void moveClawToPosition(double position) {
        position = Range.clip(position, LIFT_ARM_INSIDE, LIFT_ARM_OUTSIDE);
        liftArm.setPosition(position);
    }

    public void grabRingsWithClaw() {
        liftClaw.setPosition(LIFT_CLAW_CLOSED);
    }
    public void releaseRingsWithClaw() {
        liftClaw.setPosition(LIFT_CLAW_OPEN);
    }


    // This method unfolds the finger arm and grabs the wobble
    // Then it moves the arm to vertical position so it doesn't drag on the ground in auto, or to clear wall in endgame
    // During auto, we want to place the wobble goal touching the robot on the right side so we can immediately grab it
    // Alternative design is for hardware to allow preloading wobble inside robot
    public void pickUpWobble(double speed){//at beginning of auto or for teleop
        //set to run to position for autonomous
        wobblePickupArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //finger arm motor 0 position will be straight down, folded inside the robot when we put the robot on start line
        //open to get ready to pickup
        wobbleFinger.setPosition(WOBBLE_FINGER_OPEN);
        //wobble arm motor needs to unfold 90 degrees to horizontal position
        wobblePickupArm.setTargetPosition(WOBBLE_ARM_HORIZONTAL);
        wobblePickupArm.setPower(speed);
        //grab wobble
        wobbleFinger.setPosition(WOBBLE_FINGER_CLOSED);
        //bring the wobble arm up 180 degrees all the way up so we don't drag it
        wobblePickupArm.setTargetPosition(WOBBLE_ARM_UP);
        wobblePickupArm.setPower(speed);
    }


    // This assumes we have a wobble goal held and is held vertically
    // It lowers arm to horizontal position and drops the goal
    // Then it folds arm back into robot
    // speed determines how fast the finger arm motor moves
    public void placeWobble(double speed) {
        //set to run to position for autonomous
        wobblePickupArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //brings wobble arm down to 90 degrees
        wobblePickupArm.setTargetPosition(WOBBLE_ARM_HORIZONTAL);
        wobblePickupArm.setPower(speed);
        wobbleFinger.setPosition(WOBBLE_FINGER_OPEN);

        //reset and put Finger Arm back into the robot out of the way
        wobblePickupArm.setTargetPosition(WOBBLE_ARM_DOWN);
        wobblePickupArm.setPower(speed);
        wobbleFinger.setPosition(WOBBLE_FINGER_CLOSED);
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
        liftArm.setPosition(LIFT_ARM_OUTSIDE);
        liftClaw.setPosition(LIFT_CLAW_OPEN);
        //reset, lower lift and put claw back into robot out of the way
        wobbleLift(LIFT_BOTTOM);
        liftClaw.setPosition(LIFT_CLAW_CLOSED);
        liftArm.setPosition(LIFT_ARM_INSIDE);
    }

    // This assumes blue, for red use flip4Red when calling
    // This method gives the angle from the robot to the goal or powershot target in DEGREES
    public double calculateRobotHeadingToShoot(double targetX, double targetY) {

        double robotX = globalPosition.getXinches();
        // Tuning Tuning: Compensation for robot behavior, it shoots curved to the left, by few inches
        // Perform calculations as if the Robot center was to the left by few inches and shooting hits target straight ahead
        // Given that the Robot is directly facing the goal line (Heading = 0 (+ve X-axis)), we will also
        // actually position on the field to the right of the intended Target Y coordinate
        double robotY = globalPosition.getYinches() + ROBOT_SHOOTING_CURVE_OFFSET;
        // This code only works reliably when robot Orientation/Heading is zero, i.e. its facing +ve X-Axis direction
        // because the compensation of ROBOT_SHOOTING_CURVE_OFFSET only in Y-axis may not be accurate at other headings

        double xDiff = targetX - robotX;
        double yDiff = targetY - robotY;
        double heading = Math.atan2(yDiff, xDiff);
//        // Uncomment below for Debug printout only
//        myOpMode.telemetry.addLine("Heading to Shoot ")
//                .addData("xDiff", "%2.2f", xDiff)
//                .addData("yDiff", "%2.2f", yDiff)
//                .addData("atan2", "%2.2f", heading)
//                .addData("deg", "%2.2f", Math.toDegrees(heading));
        return Math.toDegrees(heading);
    }

    /**
     * Calculates and Returns the shooter platform title angle in degrees, to aim at the target
     *
     * @param targetX   X coordinate of target location
     * @param targetY   Y coordinate of target location
     * @param targetHeight  Height of target from the field floor
     * @return          Shooter Platform Tilt Angle in Degrees
     */
    public double calculateShooterPlatformTiltAngle(double targetX, double targetY, double targetHeight) {

        double xDiff = targetX - globalPosition.getXinches();
        double yDiff = targetY - globalPosition.getYinches();
        double distance = Math.hypot(xDiff, yDiff); //Distance on the ground

        double angleRad = Math.atan(targetHeight/distance);
        return Math.toDegrees(angleRad);
    }

    /**
     * This method allows Robot to shoot from any position on the field, and tilt the shooter platform accordingly.
     *
     * @param targetX   X coordinate of target location
     * @param targetY   Y coordinate of target location
     * @param targetHeight  Height of target from the field floor
     */
    public void tiltShooterPlatform(double targetX, double targetY, double targetHeight) {
        double angle = calculateShooterPlatformTiltAngle(targetX, targetY, targetHeight);
        // Tuning Tuning: Compensation for robot behavior, the tilt calculations need few degrees uplift
        // It may be due to gravity or physical platform tilt does not match mechanical design assumption
        tiltShooterPlatform( angle + ROBOT_PLATFORM_TILT_OFFSET );
    }

    /**
     * This method allows Robot to shoot from any position on the field, and tilt the shooter platform accordingly.
     *
     * @param tiltAngle     The angle in degrees, at which to tilt the shooting platform
     */
    public void tiltShooterPlatform(double tiltAngle) {

        shooterTiltAngleDesired = tiltAngle;    // record for telemetry printout, do not delete this line

        shooterTiltAngleClipped = Range.clip(shooterTiltAngleDesired, SHOOTER_TILT_ANGLE_MIN, SHOOTER_TILT_ANGLE_MAX);

        // MATH to convert platform tilt angle to rotation of oval supports under platform
        // scales 25 degrees (range is 20-45 degrees) of shooter platform movement to 180 degrees of oval rotation
        ovalRotationDegrees = (shooterTiltAngleClipped - SHOOTER_TILT_ANGLE_MIN) * 180 / (SHOOTER_TILT_ANGLE_MAX - SHOOTER_TILT_ANGLE_MIN);

        // Convertion of oval rotation (degrees) to motor encoder ticks
        ovalRotationTicks = (int) ((SHOOTER_OVAL_TICKS_PER_ROTATION * ovalRotationDegrees) / 360);

        angleMotor.setTargetPosition(ovalRotationTicks);
        angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angleMotor.setPower(DRIVE_SPEED_DEFAULT);

        ElapsedTime runTime = new ElapsedTime();
        while (angleMotor.isBusy() && myOpMode.opModeIsActive() && (runTime.seconds() < TIMEOUT_SHORT)){
            myOpMode.sleep(50);
        }
        //myOpMode.telemetry.addData("Shooter Tilt ", "Angle=%.2f, pos=%d in %.2fs", tiltAngle, ovalRotationTicks, runTime.seconds());

/*
        // NOTE: The code block below is used for running motors WITHOUT_ENCODER and measuring the
        // current position to determine when target position has been reached and motors should be stopped.
        // The alternate code above using motor in RUN_TO_POSITION mode is superior

        // decide direction of motor depending on current encoder position relative to target ovalRotationTicks
        // positive power values applied if current positoin pos is smaller than target and pos increments to reach target
        // negative power values applied if current position is greater than target an pos decrements to reach target
        int pos = angleMotor.getCurrentPosition();
        double signum = Math.signum(ovalRotationTicks - pos);
        angleMotor.setPower(signum * DRIVE_SPEED_SLOW);

        ElapsedTime runTime = new ElapsedTime();
        // stop when the sign changes of (difference between target and current position)
        while (((signum * (ovalRotationTicks - pos)) > 0) && (runTime.seconds() < 2.0)){
            pos = angleMotor.getCurrentPosition();
            myOpMode.telemetry.addLine("Shooter ")
                    .addData("Tilt", "%2.2f[%2.2f]", shooterTiltAngleClipped,shooterTiltAngleDesired)
                    .addData("Oval", "%2.2f",ovalRotationDegrees)
                    .addData("TPos", "%3d", ovalRotationTicks)
                    .addData("CPos", "%3d", pos);
            myOpMode.telemetry.update();
        }
        // stop the motor, platform has reached the desired title angle
        angleMotor.setPower(0.0);
*/
    }

    /**
     * This method resets the shooter platform to its resting position
     */
    public void resetShooterPlatform() {
        tiltShooterPlatform(SHOOTER_TILT_ANGLE_MIN);
        myOpMode.telemetry.addData("resetShooterPlatform()", "called");
        myOpMode.telemetry.update();
        myOpMode.sleep(1000);
    }

    public void driveToShootHighGoal() {
        // drive to desired location
        goToPosition(FieldUGoal.ORIGIN, FieldUGoal.GOALY - ROBOT_SHOOTING_CURVE_OFFSET);
        // rotate to face the goal squarely
        odometryRotateToHeading(FieldUGoal.ANGLE_POS_X_AXIS);
        // tilt platform for goal height
        tiltShooterPlatform(FieldUGoal.GOALX, FieldUGoal.GOALY, FieldUGoal.HIGH_GOAL_HEIGHT);
    }
    public void driveToShootPowerShot1() {
        // drive to desired location
        goToPosition(FieldUGoal.ORIGIN, FieldUGoal.POWERSHOT_1_Y - ROBOT_SHOOTING_CURVE_OFFSET);
        // rotate to face the goal squarely
        odometryRotateToHeading(FieldUGoal.ANGLE_POS_X_AXIS);
        // tilt platform for goal height
        tiltShooterPlatform(FieldUGoal.GOALX, FieldUGoal.POWERSHOT_1_Y, FieldUGoal.POWER_SHOT_HEIGHT);
    }
    public void driveToShootPowerShot2() {
        // drive to desired location
        goToPosition(FieldUGoal.ORIGIN, FieldUGoal.POWERSHOT_2_Y - ROBOT_SHOOTING_CURVE_OFFSET);
        // rotate to face the goal squarely
        odometryRotateToHeading(FieldUGoal.ANGLE_POS_X_AXIS);
        // tilt platform for goal height
        tiltShooterPlatform(FieldUGoal.GOALX, FieldUGoal.POWERSHOT_2_Y, FieldUGoal.POWER_SHOT_HEIGHT);
    }
    public void driveToShootPowerShot3() {
        // drive to desired location
        goToPosition(FieldUGoal.ORIGIN, FieldUGoal.POWERSHOT_3_Y - ROBOT_SHOOTING_CURVE_OFFSET);
        // rotate to face the goal squarely
        odometryRotateToHeading(FieldUGoal.ANGLE_POS_X_AXIS);
        // tilt platform for goal height
        tiltShooterPlatform(FieldUGoal.GOALX, FieldUGoal.POWERSHOT_3_Y, FieldUGoal.POWER_SHOT_HEIGHT);
    }
}

