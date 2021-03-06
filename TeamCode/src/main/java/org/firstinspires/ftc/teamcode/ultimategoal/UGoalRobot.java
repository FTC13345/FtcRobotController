package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.FtcRobotDAL;
import org.firstinspires.ftc.teamcode.drive.MecabotDrive;

import static org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.*;

public class UGoalRobot extends FtcRobotDAL {

    //constants
    static final double     INTAKE_ASMBLY_UP            = Servo.MIN_POSITION; //max is 135 degrees, all the way down
    static final double     INTAKE_ASMBLY_DOWN          = Servo.MAX_POSITION; //max is 135 degrees, all the way down
    static final double     SHOOTER_PLATFORM_POS_MIN    = Servo.MIN_POSITION;
    static final double     SHOOTER_PLATFORM_POS_MAX    = Servo.MAX_POSITION;
    static final double     RING_PUSHER_IDLE_POSITION   = Servo.MAX_POSITION;
    static final double     RING_PUSHER_SHOOT_POSITION  = Servo.MIN_POSITION;
    static final double     WOBBLE_FINGER_CLOSED        = Servo.MIN_POSITION;
    static final double     WOBBLE_FINGER_OPEN          = Servo.MAX_POSITION;
    static final double     WOBBLE_PRELOAD_CLOSED       = Servo.MIN_POSITION;
    static final double     WOBBLE_PRELOAD_OPEN         = Servo.MAX_POSITION;

    static final int        WOBBLE_ARM_TICKS_PER_REVOLUTION = (int) 1425.2*2;  // for 360 degree of rotation. 5202 Series Yellow Jacket Planetary Gear Motor encoder ticks per rev * gear ratio
    static final int        WOBBLE_ARM_ERROR_MARGIN         = 5 * WOBBLE_ARM_TICKS_PER_REVOLUTION / 360;    // 20
    static final int        WOBBLE_ARM_MAX                  = 180 * WOBBLE_ARM_TICKS_PER_REVOLUTION / 360;  // 753
    static final int        WOBBLE_ARM_UP                   = 165 * WOBBLE_ARM_TICKS_PER_REVOLUTION / 360;  // 690
    static final int        WOBBLE_ARM_RAISED               = 100 * WOBBLE_ARM_TICKS_PER_REVOLUTION / 360;  // 418
    static final int        WOBBLE_ARM_HORIZONTAL           = 75 * WOBBLE_ARM_TICKS_PER_REVOLUTION / 360;   // 313
    static final int        WOBBLE_ARM_PICKUP               = 60 * WOBBLE_ARM_TICKS_PER_REVOLUTION / 360;   // 251
    static final int        WOBBLE_ARM_DOWN                 = 0;

    //Finals
// disabled since we are using flywheelMotor without encoder
//    static final double SHOOTER_FLYWHEEL_VELO_MAX = 6000f * 28f / 60f;
//    static final double SHOOTER_FLYWHEEL_VELO_MIN = 0.3 * SHOOTER_FLYWHEEL_VELO_MAX;
//    static final double SHOOTER_FLYWHEEL_SPEED = 0.7 * SHOOTER_FLYWHEEL_VELO_MAX;
    static final double SHOOTER_FLYWHEEL_SPEED = 0.7;
    static final double SHOOTER_FLYWHEEL_STOP = 0.0;
    static final double SHOOTER_PLATFORM_ANGLE_MIN = 20.0f;
    static final double SHOOTER_PLATFORM_ANGLE_MAX = 35.0f;
    static final double SHOOTER_PLATFORM_ANGLE_OFFSET = 2.7f;

    // status variables
    double shooterFlywheelVelocity = SHOOTER_FLYWHEEL_SPEED;
    double shooterPlatformTiltAngle = SHOOTER_PLATFORM_ANGLE_MIN;
    int countRingsInHopper;

    // Motors and/or enccoders
    public DcMotor leftODwheel = null;
    public DcMotor rightODwheel = null;
    public DcMotor intakeMotor = null;
    public DcMotor wobblePickupArm = null;
    // THis is a motor driven by Spark Mini controller which takes Servo PWM input
    // Please see REV Robotics documentation about Spark Mini and example code ConceptRevSPARKMini
    public DcMotorSimple flywheelMotor = null;

    //Servos
    public Servo angleServo = null;
    public Servo ringPusher = null;
    public Servo intakeAssembly = null;
    public CRServo intakeCRServo = null;
    public Servo wobbleFinger = null;   // clamp for side wobble pickup
    public Servo wobblePreload = null;

    // Color sensor
    public NormalizedColorSensor ringSensor = null;

    // Hardware switch
    public DigitalChannel wobbleLowLimit;

    // constructor
    public UGoalRobot(HardwareMap hardwareMap, LinearOpMode opMode) {
        super(hardwareMap, opMode);
        init(hardwareMap);
    }
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
        wobblePickupArm.setDirection(DcMotor.Direction.REVERSE);
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);

        leftODwheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightODwheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobblePickupArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        // this code can be used only when flywheelMotor has an encoder port in hardware and variable is DCMotorEx
//        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        VoltageSensor batteryVoltageSensor = ahwMap.voltageSensor.iterator().next();
//        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
//                80, 0, 15, 12.3 * 12 / batteryVoltageSensor.getVoltage())
//        );

        leftODwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightODwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobblePickupArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        angleServo = ahwMap.get(Servo.class, "angleServo");
        intakeAssembly = ahwMap.get(Servo.class, "intakeAssembly");
        wobbleFinger = ahwMap.get(Servo.class, "wobbleFinger");
        wobblePreload = ahwMap.get(Servo.class, "wobblePreload");
        ringPusher = ahwMap.get(Servo.class, "ringPusherServo");
        intakeCRServo = ahwMap.get(CRServo.class, "intakeCRServo");

//        intakeAssembly.setPosition(INTAKE_ASMBLY_UP);     // disabled we dont want to reset between Auto and TeleOp
//        angleServo.setPosition(SHOOTER_PLATFORM_POS_MIN); // disabled we dont want to reset between Auto and TeleOp
        wobbleFinger.setPosition(WOBBLE_FINGER_OPEN);
        wobblePreload.setPosition(WOBBLE_PRELOAD_CLOSED);         // Clamp the preloaded wobble securely
        ringPusher.setPosition(RING_PUSHER_IDLE_POSITION);
        intakeCRServo.setDirection(CRServo.Direction.REVERSE);

        ringSensor = ahwMap.get(NormalizedColorSensor.class, "ringColorSensor");

// disabled, we are not using the limit switch
//        wobbleLowLimit = ahwMap.get(DigitalChannel.class, "wobbleLowLimit");
//        wobbleLowLimit.setMode(DigitalChannel.Mode.INPUT);
    }

    public void gameUpdate(TelemetryPacket packet) {

// disabled since we are using flywheelMotor without encoder
//        double velocity = flywheelMotor.getVelocity();
//        packet.put("upperBound", 2800);
//        packet.put("Flywheel Velocity", velocity);
//        packet.put("lowerBound", 0);
//        telemetry.addData("Flywheel Velocity", velocity);
    }

    /*
     * Ring count detection inside the hopper using the color/distance sensor
     */
    public int ringsCountInHopper() {
        /* If this color sensor also has a distance sensor, use that to count rings in the hopper
         * Note that the reported distance is only useful at very close range, and is impacted by
         * ambient light and surface reflectivity. */
        if (ringSensor instanceof DistanceSensor) {
            double distance = ((DistanceSensor) ringSensor).getDistance(DistanceUnit.CM);
            if (distance > 3.29) {
                countRingsInHopper = 0;
            } else if (distance > 3.00) {
                countRingsInHopper = 1;
            } else if (distance > 2.60) {
                countRingsInHopper = 2;
            } else if (distance > 1.80) {
                countRingsInHopper = 3;
            } else {
                countRingsInHopper = 4;
            }
        }
//        myOpMode.telemetry.addData("Rings in Hopper ", "Dist (cm) %.3f, Count %d", distance, count);
//        myOpMode.telemetry.update();
        return countRingsInHopper;
    }

    public void setLED4RingsCount() {

        switch (ringsCountInHopper()) {
            case 0:
                setBlinkinLedPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
                break;
            case 1:
                setBlinkinLedPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE);
                break;
            case 2:
                setBlinkinLedPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
                break;
            case 3:
                setBlinkinLedPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
                break;
            default:
                setBlinkinLedPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
                break;
        }
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
// disabled since we are using flywheelMotor without encoder
//        flywheelMotor.setVelocity(shooterFlywheelVelocity);
        flywheelMotor.setPower(SHOOTER_FLYWHEEL_SPEED);
    }

    public void stopShooterFlywheel() {
        flywheelMotor.setPower(SHOOTER_FLYWHEEL_STOP);
    }

    public boolean isShooterFlywheelRunning() {
        return (flywheelMotor.getPower() != SHOOTER_FLYWHEEL_STOP);
    }

    public void shooterFlywheelStepUP() {
// disabled since we are using flywheelMotor without encoder
//        if (shooterFlywheelVelocity < 0.9 * SHOOTER_FLYWHEEL_VELO_MAX ) {
//            shooterFlywheelVelocity += 50;
//            flywheelMotor.setVelocity(shooterFlywheelVelocity);
//        }
        if (shooterFlywheelVelocity < 0.9 ) {
            shooterFlywheelVelocity += 0.02;
            flywheelMotor.setPower(shooterFlywheelVelocity);
        }
    }
    public void shooterFlywheelStepDOWN() {
// disabled since we are using flywheelMotor without encoder
//        if (shooterFlywheelVelocity > 0.1 * SHOOTER_FLYWHEEL_VELO_MAX ) {
//            shooterFlywheelVelocity -= 50;
//            flywheelMotor.setVelocity(shooterFlywheelVelocity);
//        }
        if (shooterFlywheelVelocity > 0.1 ) {
            shooterFlywheelVelocity -= 0.02;
            flywheelMotor.setPower(shooterFlywheelVelocity);
        }
    }

    public void shootRing() {
        ringPusher.setPosition(RING_PUSHER_SHOOT_POSITION);
        myOpMode.sleep(150);
        ringPusher.setPosition(RING_PUSHER_IDLE_POSITION);
        myOpMode.sleep(250);
    }

    public void loadRing() {
        ringPusher.setPosition(RING_PUSHER_IDLE_POSITION);
    }

    /*
     * Wobble Preload methods
     */
    public void wobblePreloadClamp() {
        wobblePreload.setPosition(WOBBLE_PRELOAD_CLOSED);
    }
    public void wobblePreloadRelease() {
        wobblePreload.setPosition(WOBBLE_PRELOAD_OPEN);
    }

    /*
     * Wobble Pickup Arm operation methods
     */

    public void setWobbleFingerOpen(){
        wobbleFinger.setPosition(WOBBLE_FINGER_OPEN);
        myOpMode.sleep(200);    // goBilda high torque servo is slow it needs more time
    }

    public void setWobbleFingerClosed(){
        wobbleFinger.setPosition(WOBBLE_FINGER_CLOSED);
        myOpMode.sleep(300);    // goBilda high torque servo is slow it needs more time vs 150ms for high speed servo
    }

    public void moveWobbleArm(double speed) {
        wobblePickupArm.setPower(speed);
    }
    public void stopWobbleArm() {
        wobblePickupArm.setPower(MecabotDrive.DRIVE_SPEED_BRAKE);
    }

    public void setWobbleArmUp(){
        goToWobblePos(WOBBLE_ARM_UP);
    }

    public void setWobbleArmRaised(){
        goToWobblePos(WOBBLE_ARM_RAISED);
    }

    public void setWobbleArmHorizontal(){
        goToWobblePos(WOBBLE_ARM_HORIZONTAL);
    }

    public void setWobbleArmPickup(){
        goToWobblePos(WOBBLE_ARM_PICKUP);
    }

    public void setWobbleArmDown(){
        goToWobblePos(WOBBLE_ARM_DOWN);
    }

    public void goToWobblePos(int pos){
        motorRunToPosition(wobblePickupArm, pos, MecabotDrive.DRIVE_SPEED_MAX);
    }

    public void pickUpWobble(){
        setWobbleFingerClosed();
        if (myOpMode.opModeIsActive()) {
            myOpMode.sleep(250); // allow time for finger to grip the wobble
            setWobbleArmHorizontal();
        }
    }
    public void deliverWobble() {
        setWobbleFingerOpen();
    }
    public void moveWobbleArmUpwards() {
        int wobblePosition = wobblePickupArm.getCurrentPosition();
        if (wobblePosition < WOBBLE_ARM_PICKUP - WOBBLE_ARM_ERROR_MARGIN){
            setWobbleArmPickup();
        } else if (wobblePosition < WOBBLE_ARM_RAISED - WOBBLE_ARM_ERROR_MARGIN) {
            setWobbleArmRaised();
        } else if (wobblePosition < WOBBLE_ARM_UP - WOBBLE_ARM_ERROR_MARGIN) {
            setWobbleArmUp();
        }
    }

    public void moveWobbleArmDownwards() {
        int wobblePosition = wobblePickupArm.getCurrentPosition();
        if (wobblePosition > WOBBLE_ARM_RAISED + WOBBLE_ARM_ERROR_MARGIN) {
            setWobbleArmRaised();
        } else if (wobblePosition > WOBBLE_ARM_PICKUP + WOBBLE_ARM_ERROR_MARGIN) {
            setWobbleArmPickup();
        } else if (wobblePosition > WOBBLE_ARM_DOWN + UGoalRobot.WOBBLE_ARM_ERROR_MARGIN) {
            setWobbleArmDown();
        }
    }

    public void resetWobblePickupArmEncoder() {
        wobblePickupArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobblePickupArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*
     * Methods for positioning for Shooting Rings
     * Includes Shooter Platform Tilt angle, Robot Heading orientiation
     * Used for both Tele Op and Auto programs
     */
    // This assumes blue, for red use flip4Red when calling
    // This method gives the angle from the robot to the goal or powershot target in DEGREES
    public double calculateRobotHeadingToShoot(double targetX, double targetY) {

        double robotX = rrmdrive.getPoseEstimate().getX();
        // Tuning Tuning: Compensation for robot behavior, it shoots curved to the left, by few inches
        // Perform calculations as if the Robot center was to the left by few inches and shooting hits target straight ahead
        // Given that the Robot is directly facing the goal line (Heading = 0 (+ve X-axis)), we will also
        // actually position on the field to the right of the intended Target Y coordinate
        double robotY = rrmdrive.getPoseEstimate().getY() + ROBOT_SHOOTING_Y_OFFSET;
        // This code only works reliably when robot Orientation/Heading is zero, i.e. its facing +ve X-Axis direction
        // because the compensation of ROBOT_SHOOTING_Y_OFFSET only in Y-axis may not be accurate at other headings

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

    /*
     * Shooter Platform Tilt control methods
     */
    public void tiltShooterPlatform(Target target) {
        double targetY = GOALY;
        double targetHeight = POWER_SHOT_HEIGHT;
        switch(target) {
            case HIGHGOAL: targetHeight = HIGH_GOAL_HEIGHT;
                break;
            case POWERSHOT_1: targetY = POWERSHOT_1_Y;
                break;
            case POWERSHOT_2: targetY = POWERSHOT_2_Y;
                break;
            case POWERSHOT_3: targetY = POWERSHOT_3_Y;
                break;
        }
        tiltShooterPlatform(GOALX, targetY, targetHeight);
    }

    /**
     * This method tilts the shooter platform according to the detected robot current position on the field
     * for shooting rings at the specified target.
     *
     * @param targetX   X coordinate of target location
     * @param targetY   Y coordinate of target location
     * @param targetHeight  Height of target from the field floor
     */
    public void tiltShooterPlatform(double targetX, double targetY, double targetHeight) {

        tiltShooterPlatform(targetX, targetY, targetHeight, rrmdrive.getPoseEstimate().vec());
    }

    /**
     * This method tilts the shooter platform according to the specified robot position on the field
     * for shooting rings at the specified target.
     *
     * @param targetX   X coordinate of target location
     * @param targetY   Y coordinate of target location
     * @param targetHeight  Height of target from the field floor
     * @param robotPosition     X,Y coordinates of the robot on the field
     */
    public void tiltShooterPlatform(double targetX, double targetY, double targetHeight, Vector2d robotPosition) {

        double xDiff = targetX - robotPosition.getX();
        double yDiff = targetY - (robotPosition.getY());
        double distance = Math.hypot(xDiff, yDiff); //Distance on the ground

        // OPTION: Enable the code line below if/when Odometry is not working reliably and we want to ingore robot position in calculation of tilt angle.
        // WE ARE GOING TO ASSUME THAT ROBOT IS DIRECTLY FACING THE TARGET (Heading is +ve X-Axis) AND POSITIONED at Y-Axis line
        // (to be positined behind launch line), THEREFORE DISANCE TO GOAL IS SIMPLY THE X-COORDINATE OF THE TARGET
        //distance = targetX;

        double angle = Math.toDegrees(Math.atan(targetHeight/distance));

        tiltShooterPlatform( angle );
    }

    /**
     * This method allows Robot to shoot from any position on the field, and tilt the shooter platform accordingly.
     *
     * @param tiltAngle     The angle in degrees, at which to tilt the shooting platform
     */
    public void tiltShooterPlatform(double tiltAngle) {

        // Tuning Tuning: Compensation for robot behavior, the tilt calculations need few degrees uplift
        // It may be due to gravity or physical platform tilt does not match mechanical design assumption
        // add a constant offset based on field tuning
        // also record value before clipping for telemetry printout, do not add OFFSET in the Range.clip() call
        shooterPlatformTiltAngle = tiltAngle + SHOOTER_PLATFORM_ANGLE_OFFSET;

        tiltAngle = Range.clip(shooterPlatformTiltAngle, SHOOTER_PLATFORM_ANGLE_MIN, SHOOTER_PLATFORM_ANGLE_MAX);

        // MATH to convert platform tilt angle to rotation of oval supports under platform
        // scales 25 degrees (range is 20-35 degrees) of shooter platform movement to fraction of range of oval rotation
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

    public void tiltShooterPlatformUP() {
        tiltShooterPlatform(shooterPlatformTiltAngle - SHOOTER_PLATFORM_ANGLE_OFFSET + 0.5);
    }
    public void tiltShooterPlatformDOWN() {
        tiltShooterPlatform(shooterPlatformTiltAngle - SHOOTER_PLATFORM_ANGLE_OFFSET - 0.5);
    }
    public void tiltShooterPlatformMin() {
        shooterPlatformTiltAngle = SHOOTER_PLATFORM_ANGLE_MIN;
        angleServo.setPosition(SHOOTER_PLATFORM_POS_MIN);
    }

    public void tiltShooterPlatformMax() {
        shooterPlatformTiltAngle = SHOOTER_PLATFORM_ANGLE_MAX;
        angleServo.setPosition(SHOOTER_PLATFORM_POS_MAX);
    }

    //aim and shoot three rings into the high goal
    public void shootRingsIntoHighGoal(int n){
        // assumption that flywheel is already running so it can gain full speed
        // assumption that platform is already tilted for shooting at HIGH GOAL by the caller.
        // This is to allow tilting to be done while the robot is driving.

        // the pusher seems to miss once in a while so we take an extra shot to ensure 3 rings are shot
        for (int i = 0; i<n && myOpMode.opModeIsActive(); i++) {
            if (i>0) {
                myOpMode.sleep(RING_SHOOTING_INTERVAL); // allow some time for the flywheel to gain full speed after each shot
            }
            shootRing();
            setLED4RingsCount();
        }
    }

    /*****************************
     * Autonomous Program Methods using RoadRunner MecanumDrive
     ****************************/
    public void rrdriveToTarget(Target target) {
        Pose2d poseStart = rrmdrive.getPoseEstimate();
        Pose2d poseEnd = poseHighGoalTeleOp;
        switch(target) {
            case HIGHGOAL: poseEnd = poseHighGoalTeleOp;
                break;
            case POWERSHOT_1: poseEnd = posePowerShot1;
                break;
            case POWERSHOT_2: poseEnd = posePowerShot2;
                break;
            case POWERSHOT_3: poseEnd = posePowerShot3;
                break;
        }
        Trajectory goToTarget = rrmdrive.trajectoryBuilder(poseStart)
                .lineToLinearHeading(poseEnd)
                .addTemporalMarker(1.0, this::runShooterFlywheel)
                .build();
        rrmdrive.followTrajectory(goToTarget);
        // tilt shooter platform corresponding to both robot current Pose and the target
        tiltShooterPlatform(target);
    }

    public void rrdriveToDropZone(Target target) {
        Pose2d poseStart = rrmdrive.getPoseEstimate();
        Vector2d poseEnd = new Vector2d();
        switch(target) {
            case WOBBLE_LANDING_1:
                poseEnd = poseWobbleLanding1;
                break;
            case WOBBLE_LANDING_2:
                poseEnd = poseWobbleLanding2;
                break;
        }
        Trajectory goToTarget = rrmdrive.trajectoryBuilder(poseStart)
                    .splineTo(poseEnd, ANGLE_POS_Y_AXIS)
                    .build();
            rrmdrive.followTrajectory(goToTarget);
    }

    /*****************************
     * Autonomous Program Methods using MecabotDrive
     ****************************/
    public void driveToShootHighGoal() {
        // drive to desired location
        //if red reverse the y
        mcdrive.goToPosition(poseHighGoalAuto.getX(), poseHighGoalAuto.getY());
        // rotate to face the goal squarely
        mcdrive.rotateToHeading(ANGLE_POS_X_AXIS);
    }

    public void driveToShootPowerShot1() {
        // drive to desired location
        mcdrive.goToPosition(posePowerShot1.getX(), posePowerShot1.getY());
        // rotate to face the goal squarely
        mcdrive.rotateToHeading(ANGLE_POS_X_AXIS);
    }

    //instead of using go to position, use mecanum wheels to move a short distance sideways
    public void driveToNextPowerShot(){
        //move using mecanum sideways to next powershot
        //if blue, moving toward center is negative
        mcdrive.odometryMoveRightLeft(flip4Red(-DISTANCE_BETWEEN_POWERSHOT));
        // rotate to face the goal squarely
        mcdrive.rotateToHeading(ANGLE_POS_X_AXIS);
    }
    
    public void driveToShootPowerShot2() {
        // drive to desired location
        mcdrive.goToPosition(posePowerShot2.getX(), posePowerShot2.getY());
        // rotate to face the goal squarely
        mcdrive.rotateToHeading(ANGLE_POS_X_AXIS);
    }
    public void driveToShootPowerShot3() {
        // drive to desired location
        mcdrive.goToPosition(posePowerShot3.getX(), posePowerShot3.getY());
        // rotate to face the goal squarely
        mcdrive.rotateToHeading(ANGLE_POS_X_AXIS);
    }

    //Power shot 1 is furthest power shot from center
    public void shootPowerShot1(){
        tiltShooterPlatform(POWERSHOTX, flip4Red(POWERSHOT_1_Y), POWER_SHOT_HEIGHT);
        shootRing();
    }
    //Power shot 2 is in the middle
    public void shootPowerShot2(){
        tiltShooterPlatform(POWERSHOTX, flip4Red(POWERSHOT_2_Y), POWER_SHOT_HEIGHT);
        shootRing();
    }
    //Power shot 3 is closest to center
    public void shootPowerShot3(){
        tiltShooterPlatform(POWERSHOTX, flip4Red(POWERSHOT_3_Y), POWER_SHOT_HEIGHT);
        shootRing();
    }

    /*
     * Auto Drive To and Shoot Rings into the 3 Power Shots
     */
    public void goShoot3Powershot(){
        //subtract robot radius because we are using the left wheel as a guide, because shooter is a bit biased toward left
        //first powershot
        driveToShootPowerShot1();
        shootPowerShot1();
        // second powershot
        driveToNextPowerShot();
        shootPowerShot2();
        // third powershot
        driveToNextPowerShot();
        shootPowerShot3();
    }

    /*****************************
     * Telemetry setup
     ****************************/
    @Override
    protected void composeTelemetry() {
        super.composeTelemetry();
        telemetry.addLine("Tilt ")
                .addData("Angle", "%.1f°", () -> shooterPlatformTiltAngle)
                .addData("Wobble Arm", "%3d", () -> wobblePickupArm.getCurrentPosition())
                .addData("Rings", "%d", () -> countRingsInHopper);
    }
}

