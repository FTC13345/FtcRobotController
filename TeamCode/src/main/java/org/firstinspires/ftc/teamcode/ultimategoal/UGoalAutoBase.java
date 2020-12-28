package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;

import java.util.List;

/**
 * Each floor tile is 23.5 inch square (counting tabs on one side and not on the other side)
 * Each floor tile with all side tabs cut off is 22.75 inch square
 * The tabs add 0.75 to tile width on each side.
 * Field width = 23.5 * 6 - 0.75 = 70.25 each side square
 *
 * Robot is 18x18 square. Robot (x,y) position is at the center of the robot.
 */

public abstract class UGoalAutoBase extends LinearOpMode {

    // OpMode members here
    /* Declare OpMode members. */
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
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    protected String actionString = "Inactive";
    protected String message = "NO";

    /*
     * Abstract methods, must be implemented by the sub-classes
     */
    public abstract void setOdometryStartingPosition();

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
        robot = new UGoalRobot(hardwareMap, this);
        telemetry.addData(">", "Hardware initialized");
        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "WAIT for Tensorflow Ring Detection before pressing START");    //
        telemetry.update();

        // odometry is initialize inside drive system MecabotDrive class
        mcdrive = robot.getDrive();
        globalPosition = mcdrive.getOdometry();
        // this method is overridden by sub-classes to set starting coordinates for RED/BLUE side of field
        setOdometryStartingPosition();

        // start printing messages to driver station asap
        setupTelemetry();
        // initialize the image recognition for ring detection
        initRingStackDetection();
        // start ring stack detection before driver hits PLAY or STOP on driver station
        // this should be the last task in this method since we don't want to waste time in initializations when PLAY has started
        runRingStackDetection();

    }

    // for testing mainly, at the end wait for driver to press STOP, meanwhile
    // continue updating odometry position of the manual movement of the robot
    public void waitForStop() {

        shutdownRingStackDetection();
        while (opModeIsActive()) {
            telemetry.update();
        }
    }

    /**
     * do everything in autonomous mode
     * detect a skystone, pick it up, transport and delivery to the foundation,
     * move the foundation, go park itself under the skybridge
     */
    public void runFullAutoProgram() {

        // this is already set in init() but in case someone moved the robot location manually.
        setOdometryStartingPosition();

        // Start doing the tasks for points
        robot.pickUpWobble();
        encoderDriveToShootHighGoal();
        robot.shootRingsIntoHighGoal();
        encoderDriveToTargetZone(countRingStack);
        robot.deliverWobble();
        encoderDriveToPark(countRingStack);
        robot.setWobbleArmDown();
    }

    protected void setupTelemetry() {

        actionString = "Telemetry";
        drvrTelemetry.addLine("Auto ")
                .addData(getColorString(), new Func<String>() {
                    @Override
                    public String value() {
                        return getAction();
                    }
                })
                .addData("msg", new Func<String>() {
                    @Override
                    public String value() {
                        return getMessage();
                    }
                });
        drvrTelemetry.addLine("Position ")
                .addData("X", "%3.2f", new Func<Double>() {
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
                .addData("Angle", "%4.2f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return globalPosition.getOrientationDegrees();
                    }
                })
                .addData("F", new Func<String>() {
                    public String value() {
                        return mcdrive.getDirectionStr();
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

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
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
     * runRingStackDetection()
     * This method prints all the internal variables using ABC techhnology for image recognition and
     * calculating the number of rings in the stack on the floor. This is useful for development.
     * In final program this method is used only in INIT mode, not in PLAY mode
     */
    public void runRingStackDetection() {

        double timeout = -1.0f;

        actionString = "Ring Stack Detection";
        message = "Nothing";
        telemetry.addData("Ring Stack Detection", "timeout = %.1f", timeout);
        telemetry.update();
        countRingStack = detectRingStackCount(300);
        telemetry.addData("Rings Detected ", "%d", countRingStack);
        telemetry.update();

    }

    /**
     * Detects the stack of the rings and returns a string based on how many rings are in front
     * of the robot. Measures for 2 seconds and then returns what is detected
     * @param timeout Timeout value in seconds
     * @return  4, 1, or 0 depending on the number of rings
     */
    protected int detectRingStackCount(double timeout) {

        ElapsedTime time = new ElapsedTime();
        int update = 0;
        int result = 0;

        // Continue detection until driver presses PLAY or STOP button or timeout
        for (int loop = 0; !isStarted() && !isStopRequested() && time.seconds() < timeout; loop++) {
            loop++;
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    update++;
                    telemetry.addData("Time:", "%.0f ms                                                                                                                                                                                                                                                                                                     ", time.milliseconds());
                    telemetry.addData("Detection count", "%4d in %d tries", update, loop);
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i++), recognition.getLabel());
                        message = recognition.getLabel();
                        // if 1 ring in stack then W/H aspect ratio = 1.75
                        // if 4 rings in stack then W/H aspect ratio = 0.95
                        if ((recognition.getWidth() / recognition.getHeight()) > 1.3) {
                            result = 1;
                            message = "Single";
                        } else {
                            result = 4;
                            message = "Quad";
                        }
                    }
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

        return result;
    }

    public void shutdownRingStackDetection() {
        tfod.shutdown();
    }


    // using move distance methods to go to high goal
    public void encoderDriveToShootHighGoal(){
        // move around the stack
        mcdrive.encoderMoveRightLeft(12);
        mcdrive.rotateToHeading(FieldUGoal.ANGLE_POS_X_AXIS + 1);
        robot.runShooterFlywheel();
        // move back 8 inches to align with the High Goal
        // 3 tiles takes us to launch line, but robot will be half over launch line
        // so we go 2.5 tiles instead, because of the robot's radius and margin of error due to tape width
        mcdrive.encoderMoveForwardBack(FieldUGoal.TILE_LENGTH * 2.1);
        // start flywheel motor early to let it gain full speed
        mcdrive.encoderMoveRightLeft(-8);
        // we want to run the intake during shooting to drop the ring into collector, this is a good time to do it.
        robot.dropIntakeAssembly();
        // get ready into position for shooting rings
        mcdrive.rotateToHeading(FieldUGoal.ANGLE_POS_X_AXIS);
    }

    // using move distance methods to go to correct target zone
    public void encoderDriveToTargetZone(int count){
        // Assumption: We are at High Goal shooting position behind the launch line.
        // x-axis is toward/away (+/- respectively) from the goals/powershot, relative to robot forward/back
        // y-axis is toward/away (+/- respectively) from the outside wall, relative to robot left/right
        // the input inches is RELATIVE TO THE ROBOT, NOT COORDINATES
        if (count == 0){
            mcdrive.encoderMoveForwardBack(FieldUGoal.TILE_LENGTH*0.7);
            // already aligned with zone A on the X-axis, so just move on the y toward edge wall
            mcdrive.encoderMoveRightLeft(-FieldUGoal.TILE_LENGTH*0.5);
        }
        else if (count == 1){
            //we are mostly aligned with zone B on the Y-axis, so just move on the x toward goals/powershot
            mcdrive.encoderMoveForwardBack(FieldUGoal.TILE_LENGTH*1.7);
            //but better to move away on y-axis to drop wobble
            mcdrive.encoderMoveRightLeft(FieldUGoal.TILE_LENGTH*0.5);
        }
        else if(count == 4){
            // neither aligned on x nor y
            mcdrive.encoderMoveForwardBack(FieldUGoal.TILE_LENGTH*2.7);
            // move on the y same as target A, as zone A&C have the same y
            mcdrive.encoderMoveRightLeft(-FieldUGoal.TILE_LENGTH*0.5);
        }
    }
    // using move distance methods to park between powershots and goals, for easy intake of rings from human player
    // Count because we aren't using goToPosition, so we have to know which target zone we went to
    public void encoderDriveToPark(int count){
        if (count == 0){
            // for A we are one tile to the left relative to where we would be if we were in B
            // So move by one tile to the right
            mcdrive.encoderMoveRightLeft(FieldUGoal.TILE_LENGTH);
            // already on launch line, no need to move along X-Axis
        }
        else if (count == 1){
            //only move back to launch line
            mcdrive.encoderMoveForwardBack(-FieldUGoal.TILE_LENGTH);
        }
        else if (count == 4){
            // for C we are one tile to the left relative to where we would be if we were in B
            // So move by one tile to the right
            mcdrive.encoderMoveRightLeft(FieldUGoal.TILE_LENGTH);
            //move back to launch line
            mcdrive.encoderMoveForwardBack(-FieldUGoal.TILE_LENGTH*2);
        }
        // if using goToPosition, use below and we don't need the parameter
        // mcdrive.goToPosition(FieldUGoal.ROBOT_RADIUS, FieldUGoal.TILE_1_FROM_ORIGIN);

        // turn the robot around so intake faces the human player
        //mcdrive.rotateToHeading(180);
    }

}
