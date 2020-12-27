package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.MecabotMove;

public class UGoalRobot extends MecabotMove {


    public UGoalRobot(HardwareMap ahwMap, LinearOpMode opMode) {
        super(ahwMap, opMode);
        this.init(ahwMap);
    }
    // Tuning Tuning: Compensation for robot behavior, it shoots curved to the left, by few inches
    // Perform calculations as if the Robot center was to the left by few inches and shooting hits target straight ahead
    // Given that the Robot is directly facing the goal line (Heading = 0 (+ve X-axis)), we will also
    // actually position on the field to the right of the intended Target Y coordinate
    static final double     ROBOT_SHOOTING_CURVE_OFFSET = -8.5; // inches

    //constants
    static final double     INTAKE_ASMBLY_UP            = Servo.MIN_POSITION; //max is 135 degrees, all the way down
    static final double     INTAKE_ASMBLY_DOWN          = Servo.MAX_POSITION; //max is 135 degrees, all the way down
    static final double     SHOOTER_PLATFORM_POS_MIN    = Servo.MIN_POSITION;
    static final double     SHOOTER_PLATFORM_POS_MAX    = Servo.MAX_POSITION;
    static final double     RING_PUSHER_IDLE_POSITION   = Servo.MAX_POSITION;
    static final double     RING_PUSHER_SHOOT_POSITION  = Servo.MIN_POSITION;
    static final double     WOBBLE_FINGER_CLOSED        = Servo.MIN_POSITION;
    static final double     WOBBLE_FINGER_OPEN          = Servo.MAX_POSITION;
    static final double     LIFT_CLAW_OPEN              = Servo.MAX_POSITION;
    static final double     LIFT_CLAW_CLOSED            = Servo.MIN_POSITION;
    static final double     LIFT_ARM_INSIDE             = Servo.MIN_POSITION;
    static final double     LIFT_ARM_OUTSIDE            = Servo.MAX_POSITION;

    static final int        WOBBLE_ARM_TICKS_PER_REVOLUTION = 2486;  // for 360 degree of rotation. Core-hex motor encoder ticks per rev
    static final int        WOBBLE_ARM_TICKS_PER_ANGLE      = WOBBLE_ARM_TICKS_PER_REVOLUTION / 360;
    static final int        WOBBLE_ARM_UP                   = 180 * WOBBLE_ARM_TICKS_PER_ANGLE;
    static final int        WOBBLE_ARM_RELEASE_DROP_ZONE    = 90 * WOBBLE_ARM_TICKS_PER_ANGLE;
    static final int        WOBBLE_ARM_PICKUP               = 60 * WOBBLE_ARM_TICKS_PER_ANGLE;
    static final int        WOBBLE_ARM_DOWN                 = 0 * WOBBLE_ARM_TICKS_PER_ANGLE;
    static final int[]      WOBBLE_ARM_POS                  = {WOBBLE_ARM_DOWN,WOBBLE_ARM_PICKUP,WOBBLE_ARM_RELEASE_DROP_ZONE,WOBBLE_ARM_UP};

    static final int        LIFT_TOP                    = 320;
    static final int        LIFT_BOTTOM                 = 10;
    static final int        LIFT_UP_RINGS_HEIGHT        = LIFT_TOP;

    //Finals
    static final double SHOOTER_FLYWHEEL_RUN = 1.0;
    static final double SHOOTER_FLYWHEEL_STOP = 0.0;
    static final double SHOOTER_PLATFORM_ANGLE_MIN = 20.0f;
    static final double SHOOTER_PLATFORM_ANGLE_MAX = 35.0f;

    // status variables
    double shooterPlatformTiltAngle = SHOOTER_PLATFORM_ANGLE_MIN;
    // Convertion of oval rotation (degrees) to motor encoder ticks
//    int ovalRotationTicks = 0;

    // Motors and/or enccoders
    public DcMotor leftODwheel = null;
    public DcMotor rightODwheel = null;
    public DcMotor intakeMotor = null;
    public DcMotor wobblePickupArm = null;
    // THis is a motor driven by Spark Mini controller which takes Servo PWM input
    // Please see REV Robotics documentation about Spark Mini and example code ConceptRevSPARKMini
    public DcMotorSimple flywheelMotor = null;
    // This is unused due to removal of lift hardware, but we do not want to remove lot of code, so we'll declare the variable
    public DcMotor liftMotor = null;

    //Servos
    public Servo angleServo = null;
    public Servo ringPusher = null;
    public Servo intakeAssembly = null;
    public CRServo intakeCRServo = null;
    //finger is for wobble pickup, claw is for putting rings on wobble
    public Servo wobbleFinger = null;
    public Servo liftClaw = null;
    public Servo liftArm = null;

    // Initialization
    public void init(HardwareMap ahwMap) {

        leftODwheel = ahwMap.get(DcMotor.class, "leftODwheel");
        rightODwheel = ahwMap.get(DcMotor.class, "rightODwheel");
        intakeMotor = ahwMap.get(DcMotor.class, "intakeMotor");         // we are using intake motor port for cross encoder
        wobblePickupArm = ahwMap.get(DcMotor.class, "wobblePickupArm");
        flywheelMotor = ahwMap.get(DcMotorSimple.class, "flywheelMotorSparkMini");

        // direction depends on hardware installation
        leftODwheel.setDirection(DcMotor.Direction.FORWARD);
        rightODwheel.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        wobblePickupArm.setDirection(DcMotor.Direction.FORWARD);
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);

        leftODwheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightODwheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobblePickupArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftODwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightODwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobblePickupArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        angleServo = ahwMap.get(Servo.class, "angleServo");
        intakeAssembly = ahwMap.get(Servo.class, "intakeAssembly");
        wobbleFinger = ahwMap.get(Servo.class, "wobbleFinger");
        //liftClaw = ahwMap.get(Servo.class, "wobbleClaw");
        //liftArm = ahwMap.get(Servo.class, "wobbleClawArm");
        ringPusher = ahwMap.get(Servo.class, "ringPusherServo");
        intakeCRServo = ahwMap.get(CRServo.class, "intakeCRServo");

        intakeAssembly.setPosition(INTAKE_ASMBLY_UP);
        angleServo.setPosition(SHOOTER_PLATFORM_POS_MIN);
        wobbleFinger.setPosition(WOBBLE_FINGER_OPEN);
        //liftClaw.setPosition(LIFT_CLAW_OPEN);
        //liftArm.setPosition(LIFT_ARM_INSIDE);
        ringPusher.setPosition(RING_PUSHER_IDLE_POSITION);
        intakeCRServo.setDirection(CRServo.Direction.REVERSE);

        // initialize odometry subsystems
        initOdometry(leftODwheel, rightODwheel, intakeMotor);   // intake motor port is used for cross encoder
        // Set direction of odometry encoders.
        // PLEASE UPDATE THESE VALUES TO MATCH YOUR ROBOT HARDWARE *AND* the DCMOTOR DIRECTION (FORWARD/REVERSE) CONFIGURATION
        // Left encoder value, IMPORTANT: robot forward movement should produce positive encoder count
        //globalPosition.reverseLeftEncoder();
        // Right encoder value, IMPORTANT: robot forward movement should produce positive encoder count
        //globalPosition.reverseRightEncoder();
        // Perpendicular encoder value, IMPORTANT: robot right sideways movement should produce positive encoder count
        //globalPosition.reverseHorizontalEncoder();
    }

    /*
     * Ring Intake methods
     */
    public void raiseIntakeAssembly(){
        intakeAssembly.setPosition(INTAKE_ASMBLY_UP);
    }

    public void dropIntakeAssembly(){
        intakeAssembly.setPosition(INTAKE_ASMBLY_DOWN);
    }

    public void runIntake(double power) {
        intakeMotor.setPower(power);
        intakeCRServo.setPower(power);
    }

    public void stopIntake(){
        intakeCRServo.setPower(0);
        intakeMotor.setPower(0);
    }

    /**
     * Ring Shooter Flywheel and Ring Pusher Arm methods
     */
    public void runShooterFlywheel() {
        flywheelMotor.setPower(SHOOTER_FLYWHEEL_RUN);
    }

    public void stopShooterFlywheel() {
        flywheelMotor.setPower(SHOOTER_FLYWHEEL_STOP);
    }

    public boolean isShooterFlywheelRunning() {
        return (flywheelMotor.getPower() != SHOOTER_FLYWHEEL_STOP);
    }

    public void shootRing() {
        ringPusher.setPosition(RING_PUSHER_SHOOT_POSITION);
        myOpMode.sleep(150);
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

    public void runLift(double speed) {
        liftMotor.setPower(speed);
    }
    public void stopLift() {
        //stop and brake lift
        liftMotor.setPower(MOTOR_STOP_SPEED);
        // Get out of the RunMode RUN_TO_POSITION, so manual player control is possible
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void moveLift(int position) {
        position = Range.clip(position, LIFT_BOTTOM, LIFT_TOP);
        motorRunToPosition(liftMotor, position, DRIVE_SPEED_DEFAULT);
    }

    public void rotateClawInside() {
        //liftArm.setPosition(LIFT_ARM_INSIDE);
    }
    public void rotateClawOutside() {
        //liftArm.setPosition(LIFT_ARM_OUTSIDE);
    }
    public void moveClawToPosition(double position) {
        position = Range.clip(position, LIFT_ARM_INSIDE, LIFT_ARM_OUTSIDE);
        //liftArm.setPosition(position);
    }

    public void grabRingsWithClaw() {
        //liftClaw.setPosition(LIFT_CLAW_CLOSED);
    }
    public void releaseRingsWithClaw() {
        //liftClaw.setPosition(LIFT_CLAW_OPEN);
    }

    /*
     * Wobble Pickup Arm operation methods
     */

    public void setWobbleFingerOpen(){
        wobbleFinger.setPosition(WOBBLE_FINGER_OPEN);
    }

    public void setWobbleFingerClosed(){
        wobbleFinger.setPosition(WOBBLE_FINGER_CLOSED);
    }

    public void setWobbleArmUp(){
        goToWobblePos(3);
    }

    public void setWobbleArmRelease(){
        goToWobblePos(2);
    }

    public void setWobbleArmPickup(){
        goToWobblePos(1);
    }

    public void setWobbleArmDown(){
        goToWobblePos(0);
    }

    public void goToWobblePos(int pos){
        motorRunToPosition(wobblePickupArm, WOBBLE_ARM_POS[pos], MecabotMove.DRIVE_SPEED_FAST);
        myOpMode.sleep(200);
    }

    // This assumes we have a wobble goal held and is held vertically
    // It lowers arm to horizontal position and drops the goal
    // Then it folds arm back into robot
    // speed determines how fast the finger arm motor moves
    public void deliverWobble() {
        // bring wobble arm down to release
        setWobbleFingerOpen();
        // stow away the wobble arm
    }

    public void pickUpWobble(){
        setWobbleArmPickup();
        setWobbleFingerOpen();
        myOpMode.sleep(100);
        setWobbleFingerClosed();
        myOpMode.sleep(100);
        setWobbleArmRelease();
    }

    public void resetWobblePickupArmEncoder() {
        wobblePickupArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobblePickupArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*
     * Methods to lift claw with rings out of robot onto wobble goal
     */
    /**
     * liftMotor is set to run to position, and uses the parameter to determine that distance
     * constant LIFT_UP_RINGS_HEIGHT is the height we lift the claw to deposit rings onto wobble goal
     * @param height is encoder counts that tells the lift how high to go
     */
    public void wobbleLift(int height){
        liftMotor.setTargetPosition(height);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        //liftArm.setPosition(LIFT_ARM_OUTSIDE);
        //.setPosition(LIFT_CLAW_OPEN);
        //reset, lower lift and put claw back into robot out of the way
        wobbleLift(LIFT_BOTTOM);
        //liftClaw.setPosition(LIFT_CLAW_CLOSED);
        //liftArm.setPosition(LIFT_ARM_INSIDE);
    }

    /*
     * Methods for positioning for Shooting Rings
     * Includes Shooter Platform Tilt angle, Robot Heading orientiation
     * Used for both Tele Op and Auto programs
     */
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

        // TEMPORARY SINCE ODOMETRY IS NOT WORKING  RELIABLY
        // WE ARE GOING TO ASSUME THAT ROBOT IS DIRECTLY FACING THE TARGET (Heading is +ve X-Axis) AND POSITIONED at Y-Axis line (to be behind launch line)
        // THEREFORE DISANCE IS SIMPLY THE X-COORDINATE OF THE TARGET
        //distance = targetX;

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
        tiltShooterPlatform( angle );
    }

    /**
     * This method allows Robot to shoot from any position on the field, and tilt the shooter platform accordingly.
     *
     * @param tiltAngle     The angle in degrees, at which to tilt the shooting platform
     */
    public void tiltShooterPlatform(double tiltAngle) {

        shooterPlatformTiltAngle = tiltAngle;    // record for telemetry printout, do not delete this line

        tiltAngle = Range.clip(shooterPlatformTiltAngle, SHOOTER_PLATFORM_ANGLE_MIN, SHOOTER_PLATFORM_ANGLE_MAX);

        // MATH to convert platform tilt angle to rotation of oval supports under platform
        // scales 25 degrees (range is 20-45 degrees) of shooter platform movement to fraction of range of oval rotation
        double position = (tiltAngle - SHOOTER_PLATFORM_ANGLE_MIN)  / (SHOOTER_PLATFORM_ANGLE_MAX - SHOOTER_PLATFORM_ANGLE_MIN);

        angleServo.setPosition(position);

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
                    .addData("Tilt", "%2.2f[%2.2f]", shooterTiltAngleClipped,shooterPlatformTiltAngle)
                    .addData("Oval", "%2.2f",ovalRotationDegrees)
                    .addData("TPos", "%3d", ovalRotationTicks)
                    .addData("CPos", "%3d", pos);
            myOpMode.telemetry.update();
        }
        // stop the motor, platform has reached the desired title angle
        angleMotor.setPower(0.0);
*/
    }

    public void tiltShooterPlatformMin() {
        shooterPlatformTiltAngle = SHOOTER_PLATFORM_ANGLE_MIN;
        angleServo.setPosition(SHOOTER_PLATFORM_POS_MIN);
    }

    public void tiltShooterPlatformMax() {
        shooterPlatformTiltAngle = SHOOTER_PLATFORM_ANGLE_MAX;
        angleServo.setPosition(SHOOTER_PLATFORM_POS_MAX);
    }

    /*
     * Autonomous Program Methods
     */


    public void driveToShootHighGoal(FieldUGoal.AllianceColor color) {
        // drive to desired location
        //if red reverse the y
        if (color == FieldUGoal.AllianceColor.BLUE){
            goToPosition(FieldUGoal.ORIGIN, FieldUGoal.GOALY - ROBOT_SHOOTING_CURVE_OFFSET);
        }
        else{
            goToPosition(FieldUGoal.ORIGIN, -(FieldUGoal.GOALY - ROBOT_SHOOTING_CURVE_OFFSET));
        }
        // rotate to face the goal squarely
        rotateToHeading(FieldUGoal.ANGLE_POS_X_AXIS);
        // tilt platform for goal height
        tiltShooterPlatform(FieldUGoal.GOALX, FieldUGoal.GOALY, FieldUGoal.HIGH_GOAL_HEIGHT);
    }
    public void driveToShootPowerShot1(FieldUGoal.AllianceColor color) {
        // drive to desired location
        //if red reverse the y
        if (color == FieldUGoal.AllianceColor.BLUE){
            goToPosition(FieldUGoal.ORIGIN, FieldUGoal.POWERSHOT_1_Y - ROBOT_SHOOTING_CURVE_OFFSET);
        }
        else{
            goToPosition(FieldUGoal.ORIGIN, -(FieldUGoal.POWERSHOT_1_Y - ROBOT_SHOOTING_CURVE_OFFSET));
        }

        // rotate to face the goal squarely
        rotateToHeading(FieldUGoal.ANGLE_POS_X_AXIS);
        // tilt platform for goal height
        tiltShooterPlatform(FieldUGoal.GOALX, FieldUGoal.POWERSHOT_1_Y, FieldUGoal.POWER_SHOT_HEIGHT);
    }

    //instead of using go to position, use mecanum wheels to move a short distance sideways
    public void driveToNextPowerShot(FieldUGoal.AllianceColor color){
        //move using mecanum sideways to next powershot
        //if red reverse Y
        if (color == FieldUGoal.AllianceColor.BLUE){
            //if blue, moving toward center is negative, constant is already negative
            odometryMoveRightLeft(FieldUGoal.DISTANCE_BETWEEN_POWERSHOT);
        }
        else{
            //if red, moving toward center is positive, constant is already negative so needs to be negated
            odometryMoveRightLeft(-FieldUGoal.DISTANCE_BETWEEN_POWERSHOT);
        }

        // rotate to face the goal squarely
        rotateToHeading(FieldUGoal.ANGLE_POS_X_AXIS);
        // tilt platform for goal height
        tiltShooterPlatform(FieldUGoal.GOALX, FieldUGoal.POWERSHOT_1_Y, FieldUGoal.POWER_SHOT_HEIGHT);
    }


    public void driveToShootPowerShot2(FieldUGoal.AllianceColor color) {
        // drive to desired location
        //if red reverse the y
        if (color == FieldUGoal.AllianceColor.BLUE){
            goToPosition(FieldUGoal.ORIGIN, FieldUGoal.POWERSHOT_2_Y - ROBOT_SHOOTING_CURVE_OFFSET);
        }
        else{
            goToPosition(FieldUGoal.ORIGIN, -(FieldUGoal.POWERSHOT_2_Y - ROBOT_SHOOTING_CURVE_OFFSET));
        }
        // rotate to face the goal squarely
        rotateToHeading(FieldUGoal.ANGLE_POS_X_AXIS);
        // tilt platform for goal height
        tiltShooterPlatform(FieldUGoal.GOALX, FieldUGoal.POWERSHOT_2_Y, FieldUGoal.POWER_SHOT_HEIGHT);
    }
    public void driveToShootPowerShot3(FieldUGoal.AllianceColor color) {
        // drive to desired location
        //if red reverse the y
        if (color == FieldUGoal.AllianceColor.BLUE){
            goToPosition(FieldUGoal.ORIGIN, FieldUGoal.POWERSHOT_3_Y - ROBOT_SHOOTING_CURVE_OFFSET);
        }
        else{
            goToPosition(FieldUGoal.ORIGIN, -(FieldUGoal.POWERSHOT_3_Y - ROBOT_SHOOTING_CURVE_OFFSET));
        }
        // rotate to face the goal squarely
        rotateToHeading(FieldUGoal.ANGLE_POS_X_AXIS);
        // tilt platform for goal height
        tiltShooterPlatform(FieldUGoal.GOALX, FieldUGoal.POWERSHOT_3_Y, FieldUGoal.POWER_SHOT_HEIGHT);
    }
}

