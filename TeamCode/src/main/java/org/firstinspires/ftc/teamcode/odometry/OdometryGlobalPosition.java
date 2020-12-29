package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorEncoderPositionResponse;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 * Modified by Vishesh Goyal for Skystone on 12/24/2019
 * Adapted for UltimateGoal robot hardware on 10/30/2020
 *
 * All the position values are initialized to zero, including the robot orientation angle theta.
 * The implemention by Sarthak computed the change in robot angle using (left encoder count - right encoder count) indicating clockwise turn is positive angle.
 * Also the vector projection formula used for calculating global position indicates robot angle is measured from Y-Axis
 *
 * Modified by Vishesh to follow the conventional polar coordinate system, so we use this code with other code modules such as pure pursuit
 * change in robot angle = (right encoder count - left encoder count) indicating counter-clockwise rotation is positive angle value
 * Vector projection formula used for calculating global position, now uses robot angle (robotAngleRad) measured CCW from X-Axis
 *
 */
public class OdometryGlobalPosition implements Runnable {

    //Odometry wheels
    private Encoder verticalLeftEncoder, verticalRightEncoder, horizontalEncoder;
    private Thread myThread;
    private boolean isRunning = true;

    //Position variables used for storage and calculations
    private double robotGlobalX = 0, robotGlobalY = 0, robotAngleRad = Math.PI/2; // robot starts with 90 degree angle in direction of positive Y-axis
    private double verticalRightCount = 0, verticalLeftCount = 0, horizontalCount = 0;
    private double prevVerticalRightCount = 0, prevVerticalLeftCount = 0, prevHorizontalCount = 0;

    private int verticalLeftEncoderDirection = 1;
    private int verticalRightEncoderDirection = 1;
    private int horizontalEncoderDirection = 1;

    //Algorithm constants
    //The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
    public static final double ODOMETRY_ENCODER_COUNT_PER_REV = 8192;  // FTC Team 13345 Mecabot encoder hals 8192 ticks per rotation
    public static final double ODOMETRY_WHEEL_DIAMETER = 38.0f / 25.4f; // Odometry wheel has 38mm diameter, calculate in inches
    private static final double ENCODER_COUNT_PER_INCH = (ODOMETRY_ENCODER_COUNT_PER_REV) / (Math.PI * ODOMETRY_WHEEL_DIAMETER);
    private final double WHEELBASE_SEPARATION_COUNT;
    private final double HORIZONTAL_COUNT_PER_RADIAN;

    //Sleep time interval (milliseconds) for the position update thread
    private final int SLEEP_TIME = 75;  // (50-75 milliseconds is suggested)

    //Files to access the algorithm constants
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalCountPerRadianFile = AppUtil.getInstance().getSettingsFile("horizontalCountPerRadian.txt");

    /**
     * Constructor for GlobalCoordinatePosition Thread
     * @param verticalLeftEncoder left odometry encoder, facing the vertical direction
     * @param verticalRightEncoder right odometry encoder, facing the vertical direction
     * @param horizontalEncoder horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     */
    public OdometryGlobalPosition(Encoder verticalLeftEncoder, Encoder verticalRightEncoder, Encoder horizontalEncoder) {

        this.verticalLeftEncoder = verticalLeftEncoder;
        this.verticalRightEncoder = verticalRightEncoder;
        this.horizontalEncoder = horizontalEncoder;

        this.WHEELBASE_SEPARATION_COUNT = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * ENCODER_COUNT_PER_INCH;
        this.HORIZONTAL_COUNT_PER_RADIAN = Double.parseDouble(ReadWriteFile.readFile(horizontalCountPerRadianFile).trim());

        // PLEASE UPDATE THESE VALUES TO MATCH YOUR ROBOT HARDWARE *AND* the DCMOTOR DIRECTION (FORWARD/REVERSE) CONFIGURATION
        // IMPORTANT: The odometry encoders may be sharing motor ports used for other purpose which sets motor direction
        // Here we override the Encoder direction (software setting) ONLY if needed, without changing motor direction

        // Left encoder value, IMPORTANT: robot forward movement should produce positive encoder count
        //reverseLeftEncoder();
        // Right encoder value, IMPORTANT: robot forward movement should produce positive encoder count
        //reverseRightEncoder();
        // Perpendicular encoder value, IMPORTANT: robot right sideways movement should produce positive encoder count
        //reverseHorizontalEncoder();

    }

    public void start() {
        isRunning = true;
        myThread = new Thread(this);
        myThread.start();
    }

    public void stop() {
        isRunning = false;
        myThread = null;
    }

    public void resetOdometryEncoder() {
        verticalLeftEncoder.resetEncoder();
        verticalRightEncoder.resetEncoder();
        horizontalEncoder.resetEncoder();
        prevVerticalLeftCount = 0;
        prevVerticalRightCount = 0;
        prevHorizontalCount = 0;
    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void globalPositionUpdate(){
        //Get Current Positions
        verticalLeftCount = (verticalLeftEncoder.getCurrentPosition() * verticalLeftEncoderDirection);
        verticalRightCount = (verticalRightEncoder.getCurrentPosition() * verticalRightEncoderDirection);
        horizontalCount = (horizontalEncoder.getCurrentPosition()* horizontalEncoderDirection);

        double leftChange = verticalLeftCount - prevVerticalLeftCount;
        double rightChange = verticalRightCount - prevVerticalRightCount;

        //Calculate Angle
        // These formulas assume that robotAngleRad is positive when Robot is turning counter-clockwise
        // All local variables are a signed value, representing change or angle direction
        double changeInRobotAngle = (rightChange - leftChange) / (WHEELBASE_SEPARATION_COUNT);
        robotAngleRad = MathFunctions.angleWrapRad(robotAngleRad + changeInRobotAngle);

        // calculate the positional displacment only in horizontal direction
        double rawHorizontalChange = horizontalCount - prevHorizontalCount;
        // The horizontal encoder (or cross encoder) is mounted on front side of robot. A left turn (positive angle rotation) includes
        // (1) horizontal encoder travel left (negative increment) and
        // (2) swing movement to left around the center pivot of robot (positive increment).
        // The 2 terms for horizontal change must be added for net positional displacement
        // If horizontal encoder was mounted on the back of the robot then the 2nd term needed to be subtracted from 1st term.
        double horizontalChange = rawHorizontalChange + (changeInRobotAngle * HORIZONTAL_COUNT_PER_RADIAN);

        // Get the components of the motion
        // p is the vector of forward movement of the Robot, direction parallel to drivetrain wheels
        // n is the vector of normal movement of thee Robot, direction perpendicular to drivetrain wheels
        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;

        // Calculate and update the position using Vector projection formula
        // These formulas assume robotAngleRad is measured from X-Axis turning counter-clockwise, same as angle theta in a polar coordinate system
        robotGlobalX = robotGlobalX + (p*Math.cos(robotAngleRad) + n*Math.sin(robotAngleRad));
        robotGlobalY = robotGlobalY + (p*Math.sin(robotAngleRad) - n*Math.cos(robotAngleRad));

        prevVerticalLeftCount = verticalLeftCount;
        prevVerticalRightCount = verticalRightCount;
        prevHorizontalCount = horizontalCount;
    }

    /**
     * Sets or intializes the x,y coordinates and orientation angle theta of the robot global position on the FTC field
     * There is no error checking, if illegal values are passed as parameters, then subsequent behavior is undefined.
     * The field x and y coordinates can have max value of +- 70.375 inches from center of the field
     * (Calculation: Each tile is 23.5 inches square and 0.75 inch tile tabs are cut at the field edges,
     *  total 6 tiles = 140.25 inches width, add 0.25 margin from tile to wall on each side. Total 140.75 inches wall to wall)
     * The Robot x and y coordinates are at the center of the robot. For robot width of 18 inches, the robot X,Y will be 9 inches away from field wall.
     *
     * @param x X-coordinate value in inches, -61 <= x <= 61
     * @param y Y-coordinate value in inches, -61 <= y <= 61
     * @param deg orientation angle in Degrees, -180 < deg <= 180, measure from X-Axis, CCW is positive and CW is negative
     */
    public void initGlobalPosition(double x, double y, double deg) {
        robotGlobalX = x * ENCODER_COUNT_PER_INCH;
        robotGlobalY = y * ENCODER_COUNT_PER_INCH;
        robotAngleRad = Math.toRadians(deg);
    }
    /**
     * Returns the robot's global orientation in Radians unit
     * @return global orientation angle
     */
    public double getOrientationRadians() {
        return robotAngleRad;
    }

    /**
     * Returns the robot's global orientation in Degrees unit
     * @return global orientation angle
     */
    public double getOrientationDegrees() {
        return Math.toDegrees(robotAngleRad);
    }

    /**
     * Returns the robot's global x coordinate on the field in inches
     * @return global x coordinate
     */
    public double getXinches() {
        return robotGlobalX / ENCODER_COUNT_PER_INCH;
    }

    /**
     * Returns the robot's global y coordinate on the field in inches
     * @return global y coordinate
     */
    public double getYinches() {
        return robotGlobalY / ENCODER_COUNT_PER_INCH;
    }

    /**
     * Returns the robot's global x coordinate in encoder count
     * @return global x coordinate
     */
    public double getXCount() {
        return robotGlobalX;
    }

    /**
     * Returns the robot's global y coordinate in encoder count
     * @return global y coordinate
     */
    public double getYCount() {
        return robotGlobalY;
    }

    /**
     * Returns the vertical left encoder's tick count
     * @return Vertical Left Encoder count
     */
    public double getVerticalLeftCount() {
        return verticalLeftCount;
    }

    /**
     * Returns the vertical right encoder's tick count
     * @return Vertical Right Encoder count
     */
    public double getVerticalRightCount() {
        return verticalRightCount;
    }

    /**
     * Returns the horizontal encoder's tick count
     * @return Horizontal Encoder count
     */
    public double getHorizontalCount() {
        return horizontalCount;
    }

    /**
     * get the constant value of wheelbase separation, in encoder tick counts
     */
    public double getWheelbaseSeparationCount() {
        return this.WHEELBASE_SEPARATION_COUNT;
    }

    /**
     * get the constant value of horizontal encoder tick counts per Radian rotation of the Robot
     */
    public double getHorizonalCountPerRadian() {
        return this.HORIZONTAL_COUNT_PER_RADIAN;
    }

    public void reverseLeftEncoder(){
        verticalLeftEncoderDirection *= -1;
    }

    public void reverseRightEncoder(){
        verticalRightEncoderDirection *= -1;
    }

    public void reverseHorizontalEncoder(){
        horizontalEncoderDirection *= -1;
    }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            globalPositionUpdate();
            try {
                Thread.sleep(SLEEP_TIME);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
