package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;
import org.firstinspires.ftc.teamcode.odometry.MathFunctions;
import org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.AllianceColor;

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

    //** TODO: image recognition variables **//
    // private OpenCvCamera phoneCam;

    protected AllianceColor aColor;
    protected String actionString = "Inactive";
    protected String message = "NO";
    private int previousCount = 0;
    boolean haveSkystone = false;

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

    protected double flipX4Red(double value) {
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
        printRingStackDetection(0.3);
        telemetry.update();
        // stop the vision pipeline until user hits play button
        stopRingStackDetection();
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
        startRingStackDetection();
        // wait a bit for some frames to be delivered
        sleep(1000);
        // there should be several image frames collected now to detect number of rings in stack
        int count = detectRingStackCount();
        stopRingStackDetection();

        // Start moving to do rest of the work
        //** TODO: do variouss tasks in autonomous period **//

        // all done, go and Park at the end of autonomous period, add logic to choose which place to park
        parkAtInsideLane();
        // parkAtOutsideLane();
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
        telemetry.addLine("Move ")
                .addData("", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.getMovementStatus();
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
        message = "Done";
        telemetry.update();
    }

    protected void initRingStackDetection() {
        // lof of initialization setup
        // then start
        // phoneCam.startStreaming(IMG_WIDTH, IMG_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

    protected void startRingStackDetection() {
        // phoneCam.startStreaming(IMG_WIDTH, IMG_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

    protected void stopRingStackDetection() {
        // phoneCam.stopStreaming();

    }

    protected int detectRingStackCount() {
        //** TODO: Implement ring stack count detection using some image recognition technology *//
        return 1;   // hard coded until image recognition is implemented
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
        message = "Detecting";
        ElapsedTime runTime = new ElapsedTime();
        while (runTime.seconds() < timeout) {
        }
    }


    public void parkAtInsideLane() {
        actionString = "Park";
        message = String.format("start (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
        // go to middle of a tile in inside lane
        //nav.goToPosition(flipX4Red(-23), 35);
        // straighten up to travel along the X-Axis or the player alliance wall
        //nav.odometryRotateToHeading(flipAngle4Red(FieldSkystone.ANGLE_POS_X_AXIS));

        // now go park on the launch line close to center of the field
        robot.goToPosition(FieldUGoal.TILE_1_CENTER, flipX4Red(FieldUGoal.TILE_1_FROM_ORIGIN));
        message = String.format("end (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
    }

    public void parkAtOutsideLane() {
        actionString = "Park";
        message = String.format("start (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
        //
        // assumption is that starting position is approximately ( -43, 18)
        // therefore we are fairly straight angle to the outside lane
        //
        // now go park on the launch line close to center of the field
        robot.goToPosition(FieldUGoal.TILE_1_CENTER, flipX4Red(FieldUGoal.TILE_3_CENTER));
        message = String.format("end (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
    }

}

