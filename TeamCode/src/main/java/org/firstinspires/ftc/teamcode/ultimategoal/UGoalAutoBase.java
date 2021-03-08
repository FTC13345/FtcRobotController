package org.firstinspires.ftc.teamcode.ultimategoal;

import android.annotation.SuppressLint;

import java.util.Arrays;
import java.util.List;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecabotDrive;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;

import static org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.*;


public abstract class UGoalAutoBase extends LinearOpMode {

    // OpMode members here
    RRMecanumDrive rrmdrive;
    UGoalRobot robot;
    MecabotDrive mcdrive;
    int countRingStack;

    //**  image recognition variables **//
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AaWyywn/////AAABmaAngwd4NkiEkXFbiZ1MImx0IwrsJG3V9OjzrHWGA6QPXrxSP0CV2b4p72Y9v1dF1KXuTQn6Dffd9kKnvaVI6TpogRmvX8l9Z3njpjFwTmuhKY4WPXpqt3LeybKxPOEKo3vwXMy8NArm48Cqv/PfjLO5F9aCzo/U7jT738LvSBGsRuIHa+5OfohJeUwIqPDrmFb0TTysRtDsE+rbecEhs0yQqs5YJWKJ8IcOoErWDx+ba3yAvHSd51fjsXEfGNNUIkFHHHm+cLWCIIiZlj5gVSO+t4oKtDxv9Ev43NykdZASzPXiFgWSxmYDvYet48AjdMVMt6NUDOh08eAwCe+rUq0UMdqJCK6Ve4JftfakLu0S";

    /**
     * the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    protected String actionString = "Inactive";
    protected String message = "NO";

    // Roadrunner Trajectories
    Trajectory pickupWobble2;
    Trajectory shootRings1;
    Trajectory deliverWobble1;
    Trajectory deliverWobble2;
    Trajectory pickupStack;
    Trajectory pickupStack2; //if there are 4 rings, we need to pickup stack in two parts
    Trajectory shootRings2;
    Trajectory shootRings3;
    Trajectory goToPark;
    Pose2d     poseWobble1deliver = new Pose2d(TARGET_ZONE_C_X - 8, TARGET_ZONE_C_Y, ANGLE_POS_X_AXIS);
    Pose2d     poseWobble2deliver = new Pose2d(TARGET_ZONE_C_X - 6, TARGET_ZONE_C_Y - 12, ANGLE_POS_X_AXIS);


    // limit velocity constraint when we want precise positioning at end of trajectory
    TrajectoryVelocityConstraint veloc = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
            new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
    ));
    // limit acceleration constraint when we want precise positioning at end of trajectory
    TrajectoryAccelerationConstraint accelc = new ProfileAccelerationConstraint(30);

    /*
     * Abstract methods, must be implemented by the sub-classes
     */
    public abstract void setPoseStart();

    public abstract String getColorString();

    public String getAction() {
        return actionString;
    }

    public String getMessage() {
        return message;
    }

    /**
     * Initialize all hardware and software data structures
     */
    public void initializeOpMode() {

        // Initialize the robot hardware and drive system variables.
        robot = new UGoalRobot(hardwareMap, this);
        rrmdrive = robot.getRRMdrive();
        mcdrive = robot.getMCBdrive();

        // IMPORTANT: IMU must be initialized otherwise gyro reading will be always zero
        // During Tele-Op program if we want to retain the gyro heading from Autonomous program,
        // then DO NOT initialize IMU again as that resets the gyro heading to 0.0
        mcdrive.initIMU();
        rrmdrive.initIMU(); // both initIMU() do the same thing, but in future we may keep only 1 drive instance

        // Motor and Servo position initializations
        robot.wobblePreloadClamp();                         // tighten grip on the pre-loaded wobble

        // this method is overridden by sub-classes to set starting coordinates for RED/BLUE side of field
        // Ensure to set Pose after IMU initialization has been done
        setPoseStart();

        // start printing messages to driver station asap but only after hardware is initialized and odometry is running
        robot.composeTelemetry();

        // Send telemetry message to signify robot waitidng;
        telemetry.addData(">", "WAIT for Tensorflow Ring Detection before pressing START");    //
        telemetry.update();

        // trajectory building can take some time, pre-build what is possible, but for some trajectories we need to do wait until ring detection
        if (!isStopRequested()) {
            buildTrajectories();
        }
        // initialize the image recognition for ring detection
        if (!isStopRequested()) {
            initRingStackDetection();
        }
        // start ring stack detection before driver hits PLAY or STOP on driver station
        // this should be the last task in this method since we don't want to waste time in initializations when PLAY has started
        if (!isStopRequested()) {
            runRingStackDetection(600); // large timeout value so that ring detection continues between INIT and START buttons are pressed
        }
        // NOTE: The ring stack detection will continue until user presses PLAY button
        // the waitForStart() call that comes after is a formality, the code will pass through because START has been pressed to reach there

        // now that ring detection is done, calculate pose for wobble delivery that will be used for real-time trajectory building
        buildPose4TargetZone();
    }

    // for testing mainly, at the end wait for driver to press STOP, meanwhile
    // continue updating odometry position of the manual movement of the robot
    public void waitForStop() {

        shutdownRingStackDetection();
        while (opModeIsActive()) {
            telemetry.update();
        }
    }

    /*
     ****************************
     * Image Recognition Methods
     ****************************/

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    protected void initRingStackDetection() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        // Activate TensorFlow Object Detection before we wait for the start command.
        // Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }
    }

    /**
     * Detects the stack of the rings using image recognition and returns the count of rings in the
     * stack in front of the robot. Measures for timeout seconds and then returns what is detected
     * @param timeout Timeout value in seconds
     * @return  4, 1, or 0 depending on the number of rings
     */
    @SuppressLint("DefaultLocale")
    public int runRingStackDetection(double timeout) {

        actionString = "Ring Stack Detection";
        message = "Nothing";
        telemetry.update();

        ElapsedTime time = new ElapsedTime();
        int update = 0;

        // Continue detection until driver presses PLAY or STOP button or timeout
        for (int loop = 0; !isStarted() && !isStopRequested() && time.seconds() < timeout; loop++) {
            loop++;
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    update++;
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i++), recognition.getLabel());
                        // if 1 ring in stack then W/H aspect ratio = 1.75
                        // if 4 rings in stack then W/H aspect ratio = 0.95
                        if ((recognition.getWidth() / recognition.getHeight()) > 1.3) {
                            countRingStack = 1;
                            message = "Single";
                        } else {
                            countRingStack = 4;
                            message = "Quad";
                        }
                    }
                    telemetry.addData("Rings", "Count=%d [%.3f s] %4d in %d tries)",
                            countRingStack, time.seconds(), update, loop);
                    telemetry.update();
                }
            }
            else {
                telemetry.addData("TFOD IS NULL","initialization failed!!!");
                telemetry.update();
            }
            // We get about 16 frames were second = 60ms between new frames, sleep for some time
            sleep(20);
        }

        return countRingStack;
    }

    public void shutdownRingStackDetection() {
        tfod.shutdown();
    }

    /*****************************
     * Encoder Movement Methods - include a mix of Road Runner and Home Brew
     ****************************/

    public void fullAutoEncoderDrive() {
        robot.setWobbleArmPickup();
        robot.pickUpWobble();
        encoderDriveToShootHighGoal();
        robot.tiltShooterPlatform(GOALX, flip4Red(GOALY), HIGH_GOAL_HEIGHT);
        robot.shootRingsIntoHighGoal(3);
        encoderDriveToTargetZone(countRingStack);
        robot.deliverWobble();
        robot.setWobbleArmRaised();
        encoderDriveToPark(countRingStack);
        robot.setWobbleArmDown();
    }

    // using move distance methods to go to high goal
    public void encoderDriveToShootHighGoal(){
        // move around the stack
        mcdrive.encoderMoveRightLeft(12);
        mcdrive.rotateToHeading(ANGLE_POS_X_AXIS + 1);
        robot.runShooterFlywheel();
        // move back 8 inches to align with the High Goal
        // 3 tiles takes us to launch line, but robot will be half over launch line
        // so we go 2.5 tiles instead, because of the robot's radius and margin of error due to tape width
        mcdrive.encoderMoveForwardBack(TILE_LENGTH * 2.1);
        // start flywheel motor early to let it gain full speed
        mcdrive.encoderMoveRightLeft(-8);
        // we want to run the intake during shooting to drop the ring into collector, this is a good time to do it.
        robot.dropIntakeAssembly();
        // get ready into position for shooting rings
        mcdrive.rotateToHeading(ANGLE_POS_X_AXIS);
    }

    // using move distance methods to go to correct target zone
    public void encoderDriveToTargetZone(int count){
        // Assumption: We are at High Goal shooting position behind the launch line.
        // x-axis is toward/away (+/- respectively) from the goals/powershot, relative to robot forward/back
        // y-axis is toward/away (+/- respectively) from the outside wall, relative to robot left/right
        // the input inches is RELATIVE TO THE ROBOT, NOT COORDINATES
        if (count == 0){
            mcdrive.encoderMoveForwardBack(TILE_LENGTH*0.7);
            // already aligned with zone A on the X-axis, so just move on the y toward edge wall
            mcdrive.encoderMoveRightLeft(-TILE_LENGTH*0.5);
        }
        else if (count == 1){
            //we are mostly aligned with zone B on the Y-axis, so just move on the x toward goals/powershot
            mcdrive.encoderMoveForwardBack(TILE_LENGTH*1.7);
            //but better to move away on y-axis to drop wobble
            mcdrive.encoderMoveRightLeft(TILE_LENGTH*0.5);
        }
        else if(count == 4){
            // neither aligned on x nor y
            mcdrive.encoderMoveForwardBack(TILE_LENGTH*2.7);
            // move on the y same as target A, as zone A&C have the same y
            mcdrive.encoderMoveRightLeft(-TILE_LENGTH*0.5);
        }
    }
    // using move distance methods to park between powershots and goals, for easy intake of rings from human player
    // Count because we aren't using goToPosition, so we have to know which target zone we went to
    public void encoderDriveToPark(int count){
        if (count == 0){
            // for A we are one tile to the left relative to where we would be if we were in B
            // So move by one tile to the right
            mcdrive.encoderMoveRightLeft(TILE_LENGTH);
            // already on launch line, no need to move along X-Axis
        }
        else if (count == 1){
            //only move back to launch line
            mcdrive.encoderMoveForwardBack(-TILE_LENGTH);
        }
        else if (count == 4){
            // for C we are one tile to the left relative to where we would be if we were in B
            // So move by one tile to the right
            mcdrive.encoderMoveRightLeft(TILE_LENGTH);
            //move back to launch line
            mcdrive.encoderMoveForwardBack(-TILE_LENGTH*2);
        }
        // if using goToPosition, use below and we don't need the parameter
        // mcdrive.goToPosition(ROBOT_RADIUS, TILE_1_FROM_ORIGIN);

        // turn the robot around so intake faces the human player
        //mcdrive.rotateToHeading(180);
    }

    /*****************************
     * Roadrunner movement using Follow Trajectories with dead wheel odometry
     ****************************/
    private void buildPose4TargetZone() {
        double targetZoneX;
        double targetZoneY;
        switch (countRingStack) {
            case 4:
                targetZoneX = TARGET_ZONE_C_X;
                targetZoneY = TARGET_ZONE_C_Y;
                break;
            case 1:
                targetZoneX = TARGET_ZONE_B_X;
                targetZoneY = TARGET_ZONE_B_Y;
                break;
            case 0:
            default:
                targetZoneX = TARGET_ZONE_A_X;
                targetZoneY = TARGET_ZONE_A_Y;
                break;
        }

        poseWobble1deliver = new Pose2d(targetZoneX - 8, targetZoneY, ANGLE_POS_X_AXIS);
        poseWobble2deliver = new Pose2d(targetZoneX - 6, targetZoneY - 12, ANGLE_POS_X_AXIS);
    }

    enum TRAJ {
        TRAJ_LIST_START,
        SHOOT_RINGS_PRELOADED,
        RINGS_STACK_PICKUP,
        RINGS_STACK_PICKUP2,
        SHOOT_RINGS_STACK,
        SHOOT_RINGS_STACK2,
        WOBBLE_1_DELIVER,
        WOBBLE_2_PICKUP,
        WOBBLE_2_DELIVER,
        GO_PARK,
        TRAJ_LIST_END
    }

    private void buildTrajectories() {
        shootRings1 = buildTrajectory(TRAJ.SHOOT_RINGS_PRELOADED, poseStart);
        pickupStack = buildTrajectory(TRAJ.RINGS_STACK_PICKUP, shootRings1.end());
        shootRings2 = buildTrajectory(TRAJ.SHOOT_RINGS_STACK, pickupStack.end());
        pickupStack2 = buildTrajectory(TRAJ.RINGS_STACK_PICKUP2, shootRings2.end());
        shootRings3 = buildTrajectory(TRAJ.SHOOT_RINGS_STACK2, pickupStack2.end());
        deliverWobble1 = buildTrajectory(TRAJ.WOBBLE_1_DELIVER, shootRings3.end());
        pickupWobble2 = buildTrajectory(TRAJ.WOBBLE_2_PICKUP, deliverWobble1.end());
        deliverWobble2 = buildTrajectory(TRAJ.WOBBLE_2_DELIVER, pickupWobble2.end());
        goToPark = buildTrajectory(TRAJ.GO_PARK, deliverWobble2.end());
    }


    private Trajectory buildTrajectory(TRAJ traj) {
        return buildTrajectory(traj, rrmdrive.getPoseEstimate());
    }

    private Trajectory buildTrajectory(TRAJ traj, Pose2d beginPose) {
        Trajectory trajectory = null;
        switch (traj) {
            case SHOOT_RINGS_PRELOADED:
                trajectory = rrmdrive.trajectoryBuilder(beginPose)
                        .splineTo(new Vector2d(-TILE_1_FROM_ORIGIN, poseHighGoalAuto.getY()), ANGLE_POS_X_AXIS)  // swing around and avoid the ring stack
                        .splineTo(poseHighGoalAuto.vec(), poseHighGoalAuto.getHeading())  // arrive at high goal shooting position
                        .addTemporalMarker(1.0, new MarkerCallback() {
                            @Override
                            public void onMarkerReached() {
                                // on the way driving to high goal, turn on the flywheel
                                robot.runShooterFlywheel();
                                //  and tilt the platform at suitable angle, taking into account that the robot overshoots destination coordinate
                                robot.tiltShooterPlatform(GOALX, GOALY, HIGH_GOAL_HEIGHT, poseHighGoalAuto.vec().plus(new Vector2d(+1.4, 0)));
                            }
                        })
                        .build();
                break;
            case RINGS_STACK_PICKUP:
                trajectory = rrmdrive.trajectoryBuilder(beginPose, Math.toRadians(90))//robot facing forward but we want trajectory to start at a 90 angle to give enough room to turn and drive straight backward
                        .splineToConstantHeading(vecRingPickupStart, Math.toRadians(180))//We want the trajectory to be facing 180 degrees, which faces the stack rings
                        .lineTo(vecRingPickupEnd, veloc, accelc)
                        .build();
                break;
            case SHOOT_RINGS_STACK:
            case SHOOT_RINGS_STACK2:
                trajectory = rrmdrive.trajectoryBuilder(beginPose)
                        .splineTo(poseHighGoalAuto.vec(), poseHighGoalAuto.getHeading())
                        .build();
                break;
            case RINGS_STACK_PICKUP2:
                trajectory = rrmdrive.trajectoryBuilder(beginPose, Math.toRadians(90))//robot facing forward but we want trajectory to start at a 90 angle to give enough
                        .splineToConstantHeading(vecRingPickupEnd2, Math.toRadians(180))//We want the trajectory to be facing 180 degrees, which faces the stack rings
                        .build();
                break;
            case WOBBLE_1_DELIVER:
                trajectory = rrmdrive.trajectoryBuilder(beginPose)
                        .lineToLinearHeading(poseWobble1deliver)
                        .build();
                break;
            case WOBBLE_2_PICKUP:
                trajectory = rrmdrive.trajectoryBuilder(beginPose, true)
                        .splineToLinearHeading(poseWobblePickup, ANGLE_POS_Y_AXIS)
                        .addTemporalMarker(0.6, new MarkerCallback() {
                            @Override
                            public void onMarkerReached() {
                                robot.setWobbleArmPickup();
                            }
                        })
                        .build();
                break;
            case WOBBLE_2_DELIVER:
                trajectory = rrmdrive.trajectoryBuilder(beginPose)
                        .lineToLinearHeading(poseWobble2deliver)
                        .build();
                break;
            case GO_PARK:
                trajectory = rrmdrive.trajectoryBuilder(beginPose, ANGLE_NEG_Y_AXIS) // move away from the wobble goals to avoid hitting them during rotation
                        .splineToLinearHeading(posePark, ANGLE_NEG_X_AXIS)
                        .build();
                break;
        }
        return trajectory;
    }

    /**
     * do everything in autonomous mode using RoadRunner drive.
     * Assumption is that buildTrajectories() has been called during init()
     */
    public void runFullAutoProgram() {

        // this is already set in init() but in case someone moved the robot location manually.
        setPoseStart();
        robot.setLED4RingsCount();

        for (int step = 1; opModeIsActive() && (step <= 14); ++step) {
            // If countRingStack is ZERO, then skip steps 3 to 8 related to starter stack of rings
            if ((countRingStack == 0) && (step >= 3) && (step <= 9)) {
                continue;
            }
            else if ((countRingStack == 1) && (step >= 7) && (step <= 9)) {
                continue;
            }
            switch (step) {
                case 1:
                    // drive to shooting pose behind the launch line
                    rrmdrive.followTrajectory(shootRings1);
                    break;

                case 2:
                    // and tilt the platform at suitable angle for current pose
                    robot.tiltShooterPlatform(GOALX, GOALY, HIGH_GOAL_HEIGHT, rrmdrive.getPoseEstimate().vec());
                    // and shoot rings into high goal
                    robot.shootRingsIntoHighGoal(3);    // 3 preloaded rings
                    break;

                case 3:
                    robot.dropIntakeAssembly();
                    robot.runIntake(MecabotDrive.DRIVE_SPEED_MAX);
                    break;

                // Drive in reverse rings from stack on the field */
                case 4:
                    pickupStack = buildTrajectory(TRAJ.RINGS_STACK_PICKUP);
                    rrmdrive.followTrajectory(pickupStack);
                    robot.setLED4RingsCount();
                    break;

                // drive to launch line to shoot rings into high goal
                case 5:
                    shootRings2 = buildTrajectory(TRAJ.SHOOT_RINGS_STACK);
                    rrmdrive.followTrajectory(shootRings2);
                    robot.setLED4RingsCount();
                    break;
                //aim and shoot the rings into high goal
                // if only 1 ring in stack, then 2 shot attempts are enough and we have enough time for an extra shot, otherwise 3 shot attempts for 3 rings
                case 6:
                    // and tilt the platform at suitable angle for current pose
                    robot.tiltShooterPlatform(GOALX, GOALY, HIGH_GOAL_HEIGHT, rrmdrive.getPoseEstimate().vec());
                    robot.shootRingsIntoHighGoal(countRingStack == 1 ? 2 : 3);
                    break;
                //pickup the last ring in stack
                case 7:
                    pickupStack2 = buildTrajectory(TRAJ.RINGS_STACK_PICKUP2);
                    rrmdrive.followTrajectory(pickupStack2);
                    robot.setLED4RingsCount();
                    break;
                //drive to launch line to shoot rings into high goal
                case 8:
                    shootRings3 = buildTrajectory(TRAJ.SHOOT_RINGS_STACK2);
                    rrmdrive.followTrajectory(shootRings3);
                    robot.setLED4RingsCount();
                    break;
                    //aim and shoot the rings into high goal
                case 9:
                    // and tilt the platform at suitable angle for current pose
                    robot.tiltShooterPlatform(GOALX, GOALY, HIGH_GOAL_HEIGHT, rrmdrive.getPoseEstimate().vec());
                    robot.shootRingsIntoHighGoal(1);
                    robot.stopIntake();
                    break;
                case 10:
                    robot.stopShooterFlywheel();
                    break;

                // drive to Target Zone and deliver pre-loaded wobble goal
                case 11:
                    deliverWobble1 = buildTrajectory(TRAJ.WOBBLE_1_DELIVER);
                    rrmdrive.followTrajectory(deliverWobble1);
                    robot.wobblePreloadRelease();
                    break;

                // drive back to pickup 2nd wobble goal near start line
                case 12:
                    pickupWobble2 = buildTrajectory(TRAJ.WOBBLE_2_PICKUP);
                    rrmdrive.followTrajectory(pickupWobble2);
                    robot.pickUpWobble();
                    break;

                // drive to Target Zone and deliver 2nd wobble goal picked up near start line
                case 13:
                    deliverWobble2 = buildTrajectory(TRAJ.WOBBLE_2_DELIVER);
                    rrmdrive.followTrajectory(deliverWobble2);
                    robot.deliverWobble();
                    robot.setWobbleArmDown();
                    break;

                case 14:
                    goToPark = buildTrajectory(TRAJ.GO_PARK);
                    rrmdrive.followTrajectory(goToPark);
                    break;
            }
        }
    }
/*
    private void buildTrajectoriesV2() {
        // limit velocity constraint when we want precise positioning at end of trajectory
        TrajectoryVelocityConstraint veloc = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
        ));
        // limit acceleration constraint when we want precise positioning at end of trajectory
        TrajectoryAccelerationConstraint accelc = new ProfileAccelerationConstraint(30);


        pickupWobble2 = rrmdrive.trajectoryBuilder(poseStart)
                .strafeLeft(7.0).build();

        shootRings1 = rrmdrive.trajectoryBuilder(pickupWobble2.end())
                .splineTo(new Vector2d(-TILE_1_FROM_ORIGIN, 20), ANGLE_POS_X_AXIS)  // swing around and avoid the ring stack
                .splineTo(poseHighGoalAuto.vec(), ANGLE_POS_X_AXIS)  // arrive at high goal shooting position
                .addTemporalMarker(1.0, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        // on the way driving to high goal, turn on the flywheel
                        robot.runShooterFlywheel();
                        //  and tilt the platform at suitable angle
                        robot.tiltShooterPlatform(GOALX, GOALY, HIGH_GOAL_HEIGHT, poseHighGoalAuto.vec());
                    }
                })
                .build();

        double targetZoneX;
        double targetZoneY;
        switch (countRingStack) {
            case 4:
                targetZoneX = TARGET_ZONE_C_X;
                targetZoneY = TARGET_ZONE_C_Y;
                break;
            case 1:
                targetZoneX = TARGET_ZONE_B_X;
                targetZoneY = TARGET_ZONE_B_Y;
                break;
            case 0:
            default:
                targetZoneX = TARGET_ZONE_A_X;
                targetZoneY = TARGET_ZONE_A_Y;
                break;
        }

        Pose2d wobble1end = new Pose2d(targetZoneX - 3, targetZoneY - 8, ANGLE_POS_X_AXIS);
        deliverWobble1 = rrmdrive.trajectoryBuilder(shootRings1.end())
                .lineToLinearHeading(wobble1end)
                .addTemporalMarker(1.0, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        robot.setWobbleArmHorizontal();
                    }
                })
                .build();

        Pose2d wobble2end = new Pose2d(targetZoneX + 2, targetZoneY - 16, ANGLE_POS_Y_AXIS);
        deliverWobble2 = rrmdrive.trajectoryBuilder(deliverWobble1.end(), ANGLE_NEG_Y_AXIS)
                .splineToLinearHeading(wobble2end, ANGLE_POS_X_AXIS)
                .build();

        Vector2d lineupPoint = new Vector2d(ORIGIN, TILE_2_CENTER + 4);
        lineupToStack = rrmdrive.trajectoryBuilder(deliverWobble2.end(), true)
                .splineTo(lineupPoint, ANGLE_NEG_X_AXIS, veloc, accelc)        // line up precisely to pickup rings stack
                .addTemporalMarker(0.1, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        // Run intake and drive to pickup rings from ring stack
                        robot.dropIntakeAssembly();
                        robot.runIntake(MecabotDrive.DRIVE_SPEED_MAX);
                    }
                })
                .build();

        Vector2d stackPoint = new Vector2d(-TILE_1_FROM_ORIGIN, TILE_2_CENTER + 4);
        pickupStack = rrmdrive.trajectoryBuilder(lineupToStack.end(), true)
                .splineTo(stackPoint, ANGLE_NEG_X_AXIS, veloc, accelc) // location of rings stack
                .build();

        shootRings2 = rrmdrive.trajectoryBuilder(pickupStack.end())
                .splineTo(poseHighGoalAuto.vec(), ANGLE_POS_X_AXIS, veloc, accelc)
                .addTemporalMarker(0.1, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        // on the way driving to high goal, turn on the flywheel
                        // platform tilt should be already at suitable angle
                        robot.runShooterFlywheel();
                    }
                })
                .build();

        Pose2d lastTask = (countRingStack > 0) ? shootRings2.end() : deliverWobble2.end();
        goToPark = rrmdrive.trajectoryBuilder(lastTask)
                .lineToLinearHeading(posePark)
                .build();
    }

    public void runFullAutoProgramV2() {

        // this is already set in init() but in case someone moved the robot location manually.
        setPoseStart();

        if (!opModeIsActive()) {
            return;
        }

        // pickup 2nd wobble, 1st wobble is preloaded
        robot.setWobbleArmPickup();
        if (opModeIsActive()) {
            rrmdrive.followTrajectory(pickupWobble2);
        }
        robot.pickUpWobble();

        // drive to launch line and shoot rings into high goal
        if (opModeIsActive()) {
            rrmdrive.followTrajectory(shootRings1);
        }
        robot.shootRingsIntoHighGoal(3);    // 3 preloaded rings

        // drive to Target Zone and deliver the 2 wobbles
        if (opModeIsActive()) {
            rrmdrive.followTrajectory(deliverWobble1);
        }
        robot.deliverWobble();
        robot.setWobbleArmDown();
        if (opModeIsActive()) {
            rrmdrive.followTrajectory(deliverWobble2);
        }
        robot.wobblePreloadRelease();

        if (countRingStack > 0) {
            if (opModeIsActive()) {
                rrmdrive.followTrajectory(lineupToStack);
            }
            if (opModeIsActive()) {
                rrmdrive.followTrajectory(pickupStack);
            }
            // drive to launch line and shoot rings into high goal
            if (opModeIsActive()) {
                rrmdrive.followTrajectory(shootRings2);
            }
            // if only 1 ring in stack, then 2 shot attempts are enough, otherwise 3 shot attempts for 3 rings
            robot.shootRingsIntoHighGoal(countRingStack == 1 ? 2 : 3;

            robot.stopIntake();
        }
        // All done, just park at launch line which is nearby
        if (opModeIsActive()) {
            rrmdrive.followTrajectory(goToPark);
        }
    }
*/
}
