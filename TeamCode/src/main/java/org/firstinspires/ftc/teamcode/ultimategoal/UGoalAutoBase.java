package org.firstinspires.ftc.teamcode.ultimategoal;

import android.annotation.SuppressLint;

import java.util.List;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.MecabotDrive;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;

import static org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.*;


public abstract class UGoalAutoBase extends LinearOpMode {

    // OpMode members here
    /* Declare OpMode members. */
    RRMecanumDrive rrmdrive;
    UGoalRobot robot;
    MecabotDrive mcdrive;
    OdometryGlobalPosition globalPosition;
    Telemetry drvrTelemetry;
    Telemetry dashTelemetry;
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
    Trajectory pickupWobble;
    Trajectory shootRings1;
    Trajectory placeWobble1;
    Trajectory placeWobble2;
    Trajectory pickupStack;
    Trajectory shootRings2;
    Trajectory goToPark;

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
        // Redirect telemetry printouts to both Driver Station and FTC Dashboard
        dashTelemetry = FtcDashboard.getInstance().getTelemetry();
        drvrTelemetry = telemetry;
        //telemetry = new MultipleTelemetry(drvrTelemetry, dashTelemetry);

        // Initialize the robot hardware and drive system variables.
        rrmdrive = new RRMecanumDrive(hardwareMap, this);
        robot = new UGoalRobot(hardwareMap, rrmdrive, this);

        // Motor and Servo position initializations
        robot.wobblePreloadClamp();                         // tighten grip on the pre-loaded wobble
//        if (robot.wobbleLowLimit.getState() == false) {     // switch is pressed, wobble arm is at bottom
//            robot.resetWobblePickupArmEncoder();
//            telemetry.addData(">", "Wobble Arm encoder reset to ZERO");
//        }

        telemetry.addData(">", "Hardware initialized");
        // Send telemetry message to signify robot waitidng;
        telemetry.addData(">", "WAIT for Tensorflow Ring Detection before pressing START");    //
        telemetry.update();

        // odometry is initialize inside drive system MecabotDrive class
        mcdrive = robot.getDrive();
        globalPosition = mcdrive.getOdometry();
        // this method is overridden by sub-classes to set starting coordinates for RED/BLUE side of field
        setPoseStart();

        // start printing messages to driver station asap but only after hardware is initialized and odometry is running
        setupTelemetry();

        // initialize the image recognition for ring detection
        if (!isStopRequested()) {
            initRingStackDetection();
        }

        // start ring stack detection before driver hits PLAY or STOP on driver station
        // this should be the last task in this method since we don't want to waste time in initializations when PLAY has started
        if (!isStopRequested()) {
            runRingStackDetection(5); // large timeout value so that ring detection continues between INIT and START buttons are pressed
                                                // decreased timeout from 300 -> 5 in order to allow time for trajectory building after ring detection
        }
        // NOTE: The ring stack detection will continue until user presses PLAY button (Changed 5 seconds only then trajectory building)
        // the waitForStart() call that comes after is a formality, the code will pass through because START has been pressed to reach there

        // trajectory building takes 1/2 sec per trajectory, so we want to do this in init()
        if (!isStopRequested()) {
            buildTrajectories();
        }
    }

    // for testing mainly, at the end wait for driver to press STOP, meanwhile
    // continue updating odometry position of the manual movement of the robot
    public void waitForStop() {

        shutdownRingStackDetection();
        while (opModeIsActive()) {
            telemetry.update();
        }
    }

    /*****************************
     * Roadrunner movement using Follow Trajectories with dead wheel odometry
     ****************************/
    private void buildTrajectories() {
        pickupWobble = rrmdrive.trajectoryBuilder(poseStart)
                .strafeLeft(7.0).build();

        shootRings1 = rrmdrive.trajectoryBuilder(pickupWobble.end())
                .splineTo(new Vector2d(-TILE_1_FROM_ORIGIN, 20), ANGLE_POS_X_AXIS)  // swing around and avoid the ring stack
                .splineTo(poseHighGoal.vec(), ANGLE_POS_X_AXIS)  // arrive at high goal shooting position
                .addTemporalMarker(1.0, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        // on the way driving to high goal, turn on the flywheel
                        robot.runShooterFlywheel();
                        //  and tilt the platform at suitable angle
                        robot.tiltShooterPlatform(GOALX, GOALY, HIGH_GOAL_HEIGHT, poseHighGoal.vec());
                    }
                })
                .build();

        Vector2d wobble1end;
        Pose2d   wobble2end;
        switch (countRingStack) {
            case 4:
                wobble1end = new Vector2d(TARGET_ZONE_C_X - 3, TARGET_ZONE_C_Y - 8);
                wobble2end = new Pose2d(TARGET_ZONE_C_X, TARGET_ZONE_C_Y - 14, ANGLE_POS_Y_AXIS);
                break;
            case 1:
                wobble1end = new Vector2d(TARGET_ZONE_B_X - 3, TARGET_ZONE_B_Y - 8);
                wobble2end = new Pose2d(TARGET_ZONE_B_X, TARGET_ZONE_B_Y - 14, ANGLE_POS_Y_AXIS);
                break;
            case 0:
            default:
                wobble1end = new Vector2d(TARGET_ZONE_A_X - 3, TARGET_ZONE_A_Y - 8);
                wobble2end = new Pose2d(TARGET_ZONE_A_X, TARGET_ZONE_A_Y - 14, ANGLE_POS_Y_AXIS);
                break;
        }

        placeWobble1 = rrmdrive.trajectoryBuilder(shootRings1.end())
                .lineTo(wobble1end)
                .build();

        placeWobble2 = rrmdrive.trajectoryBuilder(placeWobble1.end())
                .lineToLinearHeading(wobble2end)
                .addTemporalMarker(1.0, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        robot.setWobbleArmDown();
                    }
                })
                .build();

        pickupStack = rrmdrive.trajectoryBuilder(placeWobble2.end(), true)
                .splineTo(new Vector2d(ORIGIN, GOALY), ANGLE_NEG_X_AXIS)        // align to pickup rings stack
                .splineTo(new Vector2d(-TILE_LENGTH, GOALY), ANGLE_NEG_X_AXIS)  // location of rings stack
                .addTemporalMarker(1.0, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        // Run intake and drive to pickup rings from ring stack
                        robot.runIntake(MecabotDrive.DRIVE_SPEED_MAX);
                    }
                })
                .build();

        shootRings2 = rrmdrive.trajectoryBuilder(pickupStack.end())
                .splineTo(poseHighGoal.vec(), ANGLE_POS_X_AXIS)
                .addTemporalMarker(0.1, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        // on the way driving to high goal, turn on the flywheel
                        // platform tilt should be already at suitable angle
                        robot.runShooterFlywheel();
                    }
                })
                .build();

        Pose2d lastTask = (countRingStack > 0) ? shootRings2.end() : placeWobble2.end();
        goToPark = rrmdrive.trajectoryBuilder(lastTask)
                .lineToLinearHeading(posePark)
                .build();
    }

    /**
     * do everything in autonomous mode using RoadRunner drive.
     * Assumption is that buildTrajectories() has been called during init()
     */
    public void runFullAutoProgram() {

        // this is already set in init() but in case someone moved the robot location manually.
        setPoseStart();

        if (!opModeIsActive()) {
            return;
        }

        // pickup 2nd wobble, 1st wobble is preloaded
        robot.setWobbleArmPickup();
        if (opModeIsActive()) {
            rrmdrive.followTrajectory(pickupWobble);
        }
        robot.setWobbleFingerClosed();
        if (opModeIsActive()) {
            sleep(1000); // allow time for finger to grip the wobble
        }
        robot.setWobbleArmRaised(); // raise the wobble to carry away

        // drive to launch line and shoot rings into high goal
        if (opModeIsActive()) {
            rrmdrive.followTrajectory(shootRings1);
        }
        robot.shootRingsIntoHighGoal();

        // drive to Target Zone and deliver the 2 wobbles
        if (opModeIsActive()) {
            rrmdrive.followTrajectory(placeWobble1);
        }
        robot.deliverWobbleRaiseArm();
        if (opModeIsActive()) {
            rrmdrive.followTrajectory(placeWobble2);
        }
        robot.wobblePreloadRelease();
        robot.dropIntakeAssembly();

        if (countRingStack > 0) {
            if (opModeIsActive()) {
                rrmdrive.followTrajectory(pickupStack);
            }

            // drive to launch line and shoot rings into high goal
            if (opModeIsActive()) {
                rrmdrive.followTrajectory(shootRings2);
            }
            robot.shootRingsIntoHighGoal();
            robot.stopIntake();
        }
        // All done, just park at launch line which is nearby
        if (opModeIsActive()) {
            rrmdrive.followTrajectory(goToPark);
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
                    telemetry.addData("Ring Detection", "[%.3f s] %4d in %d tries", time.seconds(), update, loop);
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i++), recognition.getLabel());
                        message = recognition.getLabel();
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
                    telemetry.addData("Rings Count ", "%d", countRingStack);
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
        robot.pickUpWobble();
        encoderDriveToShootHighGoal();
        robot.tiltShooterPlatform(GOALX, flip4Red(GOALY), HIGH_GOAL_HEIGHT);
        robot.shootRingsIntoHighGoal();
        encoderDriveToTargetZone(countRingStack);
        robot.deliverWobbleRaiseArm();
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
     * Telemetry debug printous setup
     ****************************/
    protected void setupTelemetry() {
        drvrTelemetry.addLine("Runner Position ")
                .addData("X", "%2.2f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return rrmdrive.getPoseEstimate().getX();
                    }
                })
                .addData("Y", "%2.2f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return rrmdrive.getPoseEstimate().getY();
                    }
                })
                .addData("Head", "%3.2f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return Math.toDegrees(rrmdrive.getPoseEstimate().getHeading());
                    }
                });
        drvrTelemetry.addLine("Global Position ")
                .addData("X", "%2.2f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return globalPosition.getXinches();
                    }
                })
                .addData("Y", "%2.2f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return globalPosition.getYinches();
                    }
                })
                .addData("Head", "%3.2f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return globalPosition.getOrientationDegrees();
                    }
                });
        drvrTelemetry.addLine("Odometry ")
                .addData("L", "%5.0f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return globalPosition.getVerticalLeftCount();
                    }
                })
                .addData("R", "%5.0f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return globalPosition.getVerticalRightCount();
                    }
                })
                .addData("X", "%5.0f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return globalPosition.getHorizontalCount();
                    }
                });
        drvrTelemetry.addLine("Drivetrain ")
                .addData("LF", "%5d", new Func<Integer>() {
                    @Override
                    public Integer value() {
                        return mcdrive.leftFrontDrive.getCurrentPosition();
                    }
                })
                .addData("LB", "%5d", new Func<Integer>() {
                    @Override
                    public Integer value() {
                        return mcdrive.leftBackDrive.getCurrentPosition();
                    }
                })
                .addData("RF", "%5d", new Func<Integer>() {
                    @Override
                    public Integer value() {
                        return mcdrive.rightFrontDrive.getCurrentPosition();
                    }
                })
                .addData("RB", "%5d", new Func<Integer>() {
                    @Override
                    public Integer value() {
                        return mcdrive.rightBackDrive.getCurrentPosition();
                    }
                });
        drvrTelemetry.addLine("Tilt ")
                .addData("Angle", "%.1f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return robot.shooterPlatformTiltAngle;
                    }
                })
                .addData("Pos", "%.2f",  new Func<Double>() {
                    @Override
                    public Double value() {
                        return robot.angleServo.getPosition();
                    }
                })
                .addData("Wobble Arm", "%3d", new Func<Integer>() {
                    @Override
                    public Integer value() {
                        return robot.wobblePickupArm.getCurrentPosition();
                    }
                });
        drvrTelemetry.addLine("Move ")
                .addData("", new Func<String>() {
                    @Override
                    public String value() {
                        return mcdrive.getMovementStatus();
                    }
                });
        message = "Done";
        drvrTelemetry.update();
    }
}
