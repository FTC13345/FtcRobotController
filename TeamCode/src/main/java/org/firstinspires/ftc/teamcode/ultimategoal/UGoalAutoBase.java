package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.odometry.MathFunctions;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;
import org.firstinspires.ftc.teamcode.robot.Mecabot;
import org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.AllianceColor;

import java.util.List;
import java.util.Locale;

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
    OdometryGlobalPosition globalPosition;

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

    // private OpenCvCamera phoneCam;
    protected AllianceColor aColor;
    protected String actionString = "Inactive";
    protected String message = "NO";
    private int previousCount = 0;

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

    protected double flip4Red(double value) {
        return (aColor == AllianceColor.BLUE) ? value : -value;
    }

    protected double flipAngle4Red(double value) {
        if (aColor == AllianceColor.RED) {
            value = MathFunctions.angleWrap(180 - value);
        }
        return value;
    }

    /**
     * Initialize all hardware and software data structures
     */
    public void initializeOpMode() {

        // Initialize the robot hardware and drive system variables.
        robot = new UGoalRobot(hardwareMap, this);
        telemetry.addData(">", "Hardware initialized");

        // initialize the image recognition for ring detection
        initRingStackDetection();
        // odometry is initialize inside drive system MecabotMove class
        globalPosition = robot.getPosition();
        // this method is overridden by sub-classes to set starting coordinates for RED/BLUE side of field
        setOdometryStartingPosition();
        // start the thread to calculate robot position continuously
        robot.startOdometry();
        // start printing messages to driver station asap
        setupTelemetry();
        printRingStackDetection(2.0);
        telemetry.update();

        // start the robot operator thread
        //** TODO: implement Operator class running in separate thread **//
        // oper = new RobotOperator(this, nav);
        // oper.start();
    }

    // for testing mainly, at the end wait for driver to press STOP, meanwhile
    // continue updating odometry position of the manual movement of the robot
    public void waitForStop() {

        //** TODO: implement Operator class running in separate thread **//
        // oper.stop();
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

        // start Ring stack detection counts only after play
        int count = detectRingStackCount(1.0);
        telemetry.addData("Rings Detected: ", "%d", count);

        // Start doing the tasks for points
        robot.pickUpWobble();
        driveToShootHighGoal();
        shootRingsIntoHighGoal();

        driveToTargetZone(count);
        robot.deliverWobble();

        driveToPark(count);
    }

    protected void setupTelemetry() {

        actionString = "Telemetry";
        telemetry.addLine("Auto ")
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
        telemetry.addLine("Position ")
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
                        return robot.getDirectionStr();
                    }
                });
        telemetry.addLine("Move ")
                .addData("", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.getMovementStatus();
                    }
                });
        message = "Done";
        telemetry.update();
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
     * Detects the stack of the rings and returns a string based on how many rings are in front
     * of the robot. Measures for 2 seconds and then returns what is detected
     * @param timeout Timeout value in seconds
     * @return  4, 1, or 0 depending on the number of rings
     */
    protected int detectRingStackCount(double timeout) {

        ElapsedTime time = new ElapsedTime(0);

        String label = null;
        int j = 0;

        telemetry.addData("Detecting ring stack ", "timeout = %.1f", timeout);
        while (time.milliseconds() < timeout*1000.0) {
            if (tfod != null) {
                telemetry.addData("checking tfod recognitions", "[%d]", ++j);
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    }
                    telemetry.update();
                    if (!updatedRecognitions.isEmpty()) {
                        label = updatedRecognitions.get(0).getLabel();
                    }
                }
            }

        }
        if (label == null) {
            return 0;
        }
        else if (label.equals("Quad")) {
            return 4;
        }
        else if (label.equals("Single")) {
            return 1;
        }
        else {
            return 0;
        }
    }

    public void shutdownRingStackDetection() {
        tfod.shutdown();
    }


    /**
     * printRingStackDetection()
     * This method prints all the internal variables using ABC techhnology for image recognition and
     * calculating the number of rings in the stack on the floor. This is useful for development.
     * In final program this method is used only in INIT mode, not in PLAY mode
     * CAUTION: This method does not have while (opModeIsActive()) to break out of loop when
     * user presses STOP on driver station. Using this method in play mode is dangerous
     */
    public void printRingStackDetection(double timeout) {

        actionString = "Ring Stack Detection";
        message = "Started";
        int count;
        for (int i=0;i<5;i++) {
            count = detectRingStackCount(timeout);
            message = String.format(Locale.US, "%d rings", count);
            telemetry.addData("Detected ", "%d rings on [%d] attempt", count, i);
            telemetry.update();
        }
    }

    //for auto going to and shooting the 3 power shots
    public void goShoot3Powershot(){
        //subtract robot radius because we are using the left wheel as a guide, because shooter is a bit biased toward left
        //first powershot
        robot.goToPosition(FieldUGoal.ANGLE_POS_X_AXIS, flip4Red(FieldUGoal.POWERSHOT_1_Y-robot.ROBOT_SHOOTING_CURVE_OFFSET));
        shootPowerShot1();
        robot.driveToShootPowerShot1(aColor);
        // second powershot
        robot.driveToNextPowerShot(aColor);
        // third powershot
        robot.driveToNextPowerShot(aColor);
    }

    // using move distance methods to go to high goal
    public void driveToShootHighGoal(){
        // move around the stack
        robot.encoderMoveRightLeft(15);
        //robot.rotateToHeading(FieldUGoal.ANGLE_POS_X_AXIS);
        // 3 tiles takes us to launch line, but robot will be half over launch line
        // so we go 2.5 tiles instead, because of the robot's radius and margin of error due to tape width
        robot.encoderMoveForwardBack(FieldUGoal.TILE_LENGTH * 2.4);
        // move back 8 inches because we moved out of the way of the stack
        robot.encoderMoveRightLeft(-8);
        // we want to run the intake during shooting to drop the ring into collector, this is a good time to do it.
        robot.dropIntakeAssembly();
        // get ready into position for shooting rings
        robot.rotateToHeading(FieldUGoal.ANGLE_POS_X_AXIS);

    }
    // using move distance methods to go to correct target zone
    public void driveToTargetZone(int count){
        // Assumption: We are at High Goal shooting position behind the launch line.
        // x-axis is toward/away (+/- respectively) from the goals/powershot, relative to robot forward/back
        // y-axis is toward/away (+/- respectively) from the outside wall, relative to robot left/right
        // the input inches is RELATIVE TO THE ROBOT, NOT COORDINATES
        if (count == 0){
            robot.encoderMoveForwardBack(Mecabot.HALF_WIDTH);
            // already aligned with zone A on the X-axis, so just move on the y toward edge wall
            robot.encoderMoveRightLeft(-FieldUGoal.TILE_LENGTH/2);
        }
        else if (count == 1){
            //we are already aligned with zone B on the Y-axis, so just move on the x toward goals/powershot
            robot.encoderMoveForwardBack(FieldUGoal.TILE_LENGTH*1.5);
        }
        else if(count == 4){
            // neither aligned on x nor y
            robot.encoderMoveForwardBack(FieldUGoal.TILE_LENGTH*2.5);
            // move on the y same as target A, as zone A&C have the same y
            robot.encoderMoveRightLeft(-FieldUGoal.TILE_LENGTH/2);
        }
    }
    // using move distance methods to park between powershots and goals, for easy intake of rings from human player
    // Count because we aren't using goToPosition, so we have to know which target zone we went to
    public void driveToPark(int count){
        if (count == 0){
            // if A  we are half a tile to the left relative to where we would be if we were in B
            // So move an additional half tile to the right
            robot.encoderMoveRightLeft(FieldUGoal.TILE_LENGTH);
        }
        else if (count == 1){
            //if target zone b, just move half a tile to the right to align with human player
            robot.encoderMoveRightLeft(FieldUGoal.TILE_LENGTH/2);
            //move back to launch line
            robot.encoderMoveForwardBack(-FieldUGoal.TILE_LENGTH);
        }
        else if (count == 4){
            // if A  we are half a tile to the left relative to where we would be if we were in B
            // So move an additional half tile to the right
            robot.encoderMoveRightLeft(FieldUGoal.TILE_LENGTH);
            //move back to launch line
            robot.encoderMoveForwardBack(-FieldUGoal.TILE_LENGTH*2);
        }


        // if using goToPosition, use below and we don't need the parameter
        // robot.goToPosition(FieldUGoal.ROBOT_RADIUS, FieldUGoal.TILE_1_FROM_ORIGIN);

        // turn the robot around so intake faces the human player
        robot.rotateToHeading(180);
    }








    //Power shot 1 is furthest power shot from center
    public void shootPowerShot1(){
        robot.rotateToHeading(FieldUGoal.ANGLE_POS_X_AXIS);
        robot.tiltShooterPlatform(FieldUGoal.POWERSHOTX, flip4Red(FieldUGoal.POWERSHOT_1_Y), FieldUGoal.POWER_SHOT_HEIGHT);
        robot.shootRing();
    }
    //Power shot 2 is in the middle
    public void shootPowerShot2(){
        robot.rotateToHeading(FieldUGoal.ANGLE_POS_X_AXIS);
        robot.tiltShooterPlatform(FieldUGoal.POWERSHOTX, flip4Red(FieldUGoal.POWERSHOT_2_Y), FieldUGoal.POWER_SHOT_HEIGHT);
        robot.shootRing();
    }
    //Power shot 3 is closest to center
    public void shootPowerShot3(){
        robot.rotateToHeading(FieldUGoal.ANGLE_POS_X_AXIS);
        robot.tiltShooterPlatform(FieldUGoal.POWERSHOTX, flip4Red(FieldUGoal.POWERSHOT_3_Y), FieldUGoal.POWER_SHOT_HEIGHT);
        robot.shootRing();
    }

    //aim and shoot three rings into the high goal
    public void shootRingsIntoHighGoal(){
        robot.runShooterFlywheel();
        //run intake for third ring
        robot.runIntake(1.0);
        robot.tiltShooterPlatform(FieldUGoal.GOALX, flip4Red(FieldUGoal.GOALY), FieldUGoal.HIGH_GOAL_HEIGHT);
        for (int i = 0; i<5; i++) {
            sleep(800); // allow 1 sec for the flywheel to gain full speed after each shot
            robot.shootRing();
        }
        robot.stopIntake();
        robot.stopShooterFlywheel();
    }

    // distance between center of robot and where wobble is placed is 15 inches
    // 8.5 is robot radius and 6.5 is length of wobble finger arm
    public void deliverWobbleAtTargetZone(int count){
        if (count == 0){
            // We are already in position to drop the wobble over target zone A, so we don't need to move

            //robot.goToPosition(FieldUGoal.TARGET_ZONE_A_X,flip4Red(FieldUGoal.TARGET_ZONE_A_Y - 15));
        }
        else if (count == 1){
            robot.goToPosition(FieldUGoal.TARGET_ZONE_B_X,flip4Red(FieldUGoal.TARGET_ZONE_B_Y - 15));
        }
        else if(count == 4){
            robot.goToPosition(FieldUGoal.TARGET_ZONE_C_X,flip4Red(FieldUGoal.TARGET_ZONE_C_Y - 15));
        }
        else {
            telemetry.addData("detectRingStackCount did not return 0, 1, or 4", "");
        }
        //Wobble is delivered on left side, for red target zones, we need to turn robot to deliver on the right side
        if (aColor == AllianceColor.RED){
            robot.rotateToHeading(FieldUGoal.ANGLE_NEG_X_AXIS);
        }
        robot.deliverWobble();

    }
    // if we are blue, we will be facing backward, and need to reverse heading to not have to turn
    // inside lane is the point on the launch line right between power goal and power shot
    public void parkAtInsideLane(int count) {
        actionString = "Park";
        message = String.format("start (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
        //if count is 0, stay in place because we are already over launch line
        if (count == 0){
            //do nothing
        }
        //if in other target zones, just drive directly
        else { // now go park on the launch line close to center of the field
            robot.goToPosition(FieldUGoal.TILE_1_CENTER, flip4Red(FieldUGoal.TILE_1_FROM_ORIGIN));
            message = String.format("end (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
        }
    }
    // if we are blue, we will be facing backward, and need to reverse heading to not have to turn
    // outside lane is the point on the launch line at the center of the tile on the edge of the field.
    public void parkAtOutsideLane(int count) {
        actionString = "Park";
        message = String.format("start (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
        //if count is 0, stay in place because we are already over launch line
        if (count == 0){
            //do nothing
        }
        //if in other target zones, just drive directly
        else { // now go park on the launch line close to center of the field
            robot.goToPosition(FieldUGoal.TILE_1_CENTER, flip4Red(FieldUGoal.TILE_3_CENTER));
            message = String.format("end (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
        }
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

}
