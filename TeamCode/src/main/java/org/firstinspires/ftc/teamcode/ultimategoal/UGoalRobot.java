package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    //constants
    static final double     INTAKE_DOWN_ANGLE           = Servo.MAX_POSITION; //max is 135 degrees, all the way down
    static final double     RING_PUSHER_IDLE_POSITION   = Servo.MAX_POSITION;
    static final double     RING_PUSHER_SHOOT_POSITION  = Servo.MIN_POSITION;
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
    public DcMotor wobbleFingerArm = null;
    // THis is a motor driven by Spark Mini controller which takes Servo PWM input
    // Please see REV Robotics documentation about Spark Mini and example code ConceptRevSPARKMini
    public DcMotorSimple flyWheelMotor = null;

    //Servos
    public Servo ringPusher = null;
    public Servo releaseIntake = null;
    //finger is for wobble pickup, claw is for putting rings on wobble
    public Servo wobbleFinger = null;
    public Servo wobbleClaw = null;
    public Servo wobbleClawArm = null;

    // Initialization
    public void init(HardwareMap ahwMap) {

        angleMotor = ahwMap.get(DcMotor.class, "angleMotor");
        intakeMotor = ahwMap.get(DcMotor.class, "intakeMotor");
        wobbleFingerArm = ahwMap.get(DcMotor.class, "wobbleFingerArm");
        liftMotor = ahwMap.get(DcMotor.class, "liftMotor");
        flyWheelMotor = ahwMap.get(DcMotorSimple.class, "launcherMotorSparkMini");

        // direction depends on hardware installation
        angleMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        wobbleFingerArm.setDirection(DcMotor.Direction.REVERSE);
        flyWheelMotor.setDirection(DcMotor.Direction.REVERSE);

        angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbleFingerArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        angleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleFingerArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        releaseIntake = ahwMap.get(Servo.class, "releaseIntake");
        wobbleFinger = ahwMap.get(Servo.class, "wobbleFinger");
        wobbleClaw = ahwMap.get(Servo.class, "wobbleClaw");
        wobbleClawArm = ahwMap.get(Servo.class, "wobbleClawArm");
        ringPusher = ahwMap.get(Servo.class, "launcherServo");

        // releaseIntake should NOT be initialized to any specific position.
        // we want to be able to initialize robot regardless whether intake is lifted up or let down
        wobbleFinger.setPosition(WOBBLE_FINGER_CLOSED);
        wobbleClaw.setPosition(WOBBLE_CLAW_OPEN);
        wobbleClawArm.setPosition(WOBBLE_CLAW_ARM_INSIDE);
        ringPusher.setPosition(RING_PUSHER_IDLE_POSITION);
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
        liftMotor.setPower(0);
    }

    //only for autonomous, for teleop use function that doens't call this
    // This assumes blue, for red use flip4Red when calling
    // This method gives the angle from the robot to the goal or powershot target in Radians
    public double calculateRobotHeadingToShoot(double robotX, double robotY, double targetX, double targetY) {

        double xDiff = targetX - robotX;
        double yDiff = targetY - robotY;

        return Math.atan2(xDiff, yDiff);
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
        tiltShooterPlatform( calculateShooterPlatformTiltAngle(targetX, targetY, targetHeight) );
    }

    /**
     * This method allows Robot to shoot from any position on the field, and tilt the shooter platform accordingly.
     *
     * @param tiltAngle     The angle in degrees, at which to tilt the shooting platform
     */
    public void tiltShooterPlatform(double tiltAngle) {

        shooterTiltAngleDesired = tiltAngle;    // record for telemetry printout

        shooterTiltAngleClipped = Range.clip(shooterTiltAngleDesired, SHOOTER_TILT_ANGLE_MIN, SHOOTER_TILT_ANGLE_MAX);

        // MATH to convert platform tilt angle to rotation of oval supports under platform
        // scales 25 degrees (range is 20-45 degrees) of shooter platform movement to 180 degrees of oval rotation
        ovalRotationDegrees = (shooterTiltAngleClipped - SHOOTER_TILT_ANGLE_MIN) * 180 / (SHOOTER_TILT_ANGLE_MAX - SHOOTER_TILT_ANGLE_MIN);

        // Convertion of oval rotation (degrees) to motor encoder ticks
        ovalRotationTicks = (int) ((SHOOTER_OVAL_TICKS_PER_ROTATION * ovalRotationDegrees) / 360);

        // decide direction of motor depending on current encoder position relative to target ovalRotationTicks
        // positive power values applied if current positoin pos is smaller than target and pos increments to reach target
        // negative power values applied if current position is greater than target an pos decrements to reach target
        int pos = leftFrontDrive.getCurrentPosition();
        double signum = Math.signum(ovalRotationTicks - pos);
        angleMotor.setPower(signum * DRIVE_SPEED_SLOW);

        ElapsedTime runTime = new ElapsedTime();
        // stop when the sign changes of (difference between target and current position)
        while (((signum * (ovalRotationTicks - pos)) > 0) && (runTime.seconds() < 2.0)){
            pos = leftFrontDrive.getCurrentPosition();
            myOpMode.telemetry.addLine("Shooter ")
                    .addData("Tilt", "%2.2f[%2.2f]", shooterTiltAngleClipped,shooterTiltAngleDesired)
                    .addData("Oval", "%2.2f",ovalRotationDegrees)
                    .addData("TPos", "%3d", ovalRotationTicks).addData("CPos", "%3d", pos);
            myOpMode.telemetry.update();
        }
        angleMotor.setPower(0.0);
    }

    /**
     * This method resets the shooter platform to its resting position
     */
    public void resetShooterPlatform() {
        tiltShooterPlatform(SHOOTER_TILT_ANGLE_MIN);
        myOpMode.telemetry.addData("resetShooterPlatform()", "called");
        myOpMode.telemetry.update();
        myOpMode.sleep(5000);
    }

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

    /**
     * Move the ring pusher arm to its default resting position (not pushing the ring just yet)
     */
    public void loadRing() {
        ringPusher.setPosition(RING_PUSHER_IDLE_POSITION);
    }

    public void releaseIntake(){
        releaseIntake.setPosition(INTAKE_DOWN_ANGLE);
    }

    public void runIntake(double power) {
        intakeMotor.setPower(power);
    }

    public void stopIntake(){
        intakeMotor.setPower(0);
    }
}

