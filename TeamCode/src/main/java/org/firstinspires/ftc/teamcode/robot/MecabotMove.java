package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;
import org.firstinspires.ftc.teamcode.odometry.MathFunctions;

import java.util.Locale;


public class MecabotMove extends Mecabot {

    enum WheelPosition { LEFT_FRONT, LEFT_BACK, RIGHT_FRONT, RIGHT_BACK }
    public enum DriveType {TANK, MECANUM, DIAGONAL}

    // Encoder based movement calculation constants
    static final double ENCODER_TICKS_PER_ROTATION  = 537.6f; // goBilda 5202 series Yellow Jacket Planetary 19.2:1 gear ratio, 312 RPM
    static final float  mmPerInch                   = 25.4f;
    static final double WHEEL_DIA                   = 100 / mmPerInch;  // goBilda mecanum wheels 1st gen part# 3213-3606-0001
    static final double WHEEL_CIRCUMFERENCE         = Math.PI * WHEEL_DIA;
    static final double ENCODER_TICKS_PER_INCH      = ENCODER_TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE;
    static final int    ENCODER_TICKS_ERR_MARGIN    = 50;
    static final double OUTER_TO_INNER_TURN_SPEED_RATIO = 6.0;

    // Driving speeds
    public static final double DRIVE_SPEED_BRAKE   = 0.0;
    public static final double DRIVE_SPEED_MIN     = 0.2;
    public static final double DRIVE_SPEED_SLOW    = 0.3;
    public static final double DRIVE_SPEED_DEFAULT = 0.6;
    public static final double DRIVE_SPEED_FAST    = 0.8;
    public static final double DRIVE_SPEED_MAX     = 1.0;
    public static final double ROTATE_SPEED_MIN    = 0.12;
    public static final double ROTATE_SPEED_SLOW   = 0.2;
    public static final double ROTATE_SPEED_DEFAULT= 0.4;
    public static final double ROTATE_SPEED_FAST   = 0.5;
    static final double DEFAULT_SPEED       = 0.6;  //default wheel speed, same as motor power

    // Distance and Time thresholds
    public static final double DIST_MARGIN          = 1.0; // inches
    public static final double DIST_NEAR            = 4.0; // inches
    public static final double DIST_SLOWDOWN        = 16.0; // inches
    public static final double TIMEOUT_LONG         = 5.0; // seconds
    public static final double TIMEOUT_DEFAULT      = 3.0; // seconds
    public static final double TIMEOUT_ROTATE       = 2.0; // seconds
    public static final double TIMEOUT_SHORT        = 1.5; // seconds
    public static final double TIMEOUT_QUICK        = 1.0; // seconds

    // member variables for state
    protected Mecabot             robot;          // Access to the Robot hardware
    protected LinearOpMode        myOpMode;       // Access to the OpMode object
    protected OdometryGlobalPosition globalPosition; // Robot global position tracker
    private   Thread              globalPositionThread;
    protected String              movementStatus          = "";

    //The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
    public static final double ODOMETRY_ENCODER_COUNT_PER_ROTATION = 1440;  // FTC Team 13345 Mecabot encoder has 1440 ticks per rotation
    public static final double ODOMETRY_WHEEL_DIAMETER = 38.0f / 25.4f; // Odometry wheel has 38mm diameter, calculate in inches

    // odometry encoder wheels
    public DcMotor leftEncoder = null;
    public DcMotor rightEncoder = null;
    public DcMotor crossEncoder = null;

    /* Constructor */
    public MecabotMove(HardwareMap ahwMap, LinearOpMode opMode) {
        super(ahwMap);
        // Save reference to OpMode and Hardware map
        myOpMode = opMode;
        robot = this;
    }

    // Access methods
    public OdometryGlobalPosition getPosition() {
        return globalPosition;
    }

    public OdometryGlobalPosition initOdometry(DcMotor leftODencoder, DcMotor rightODEncoder, DcMotor crossODEncoder) throws IllegalStateException {

        // Initialize Odometry encoders
        leftEncoder = leftODencoder;
        rightEncoder = rightODEncoder;
        crossEncoder = crossODEncoder;

        if ((leftEncoder == null) || (rightEncoder == null) || (crossEncoder == null)) {
            throw new IllegalStateException("Mecabot hardware must be initialized before Odometry.");
        }

        //Create and start GlobalPosition thread to constantly update the global position coordinates.
        globalPosition = new OdometryGlobalPosition(leftEncoder, rightEncoder, crossEncoder, ODOMETRY_ENCODER_COUNT_PER_ROTATION, ODOMETRY_WHEEL_DIAMETER);

        // IMPORTANT NOTE: The following configuration should ideally be done by the caller of this method because the motors that are passed as arguments
        // may have a FORWARD or REVERSE direction depending on the function served by those motors. The odometry encoders are usually simply using the
        // motor ports without any relation to those motor functionality.

        // Set direction of odometry encoders.
        // PLEASE UPDATE THESE VALUES TO MATCH YOUR ROBOT HARDWARE *AND* the DCMOTOR DIRECTION (FORWARD/REVERSE) CONFIGURATION
        // Left encoder value, IMPORTANT: robot forward movement should produce positive encoder count
        //globalPosition.reverseLeftEncoder();
        // Right encoder value, IMPORTANT: robot forward movement should produce positive encoder count
        //globalPosition.reverseRightEncoder();
        // Perpendicular encoder value, IMPORTANT: robot right sideways movement should produce positive encoder count
        //globalPosition.reverseHorizontalEncoder();

        myOpMode.telemetry.addData("Wheelbase Separation", globalPosition.getWheelbaseSeparationCount());
        myOpMode.telemetry.addData("Horizontal Count Per Radian", globalPosition.getHorizonalCountPerRadian());

        return globalPosition;
    }

    public void startOdometry() {
        globalPositionThread = new Thread(globalPosition);
        globalPositionThread.start();
    }

    public void stopOdometry() {
        globalPosition.stop();
    }

    public void resetOdometryEncoder() {
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        crossEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        crossEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public String getMovementStatus() {
        return movementStatus;
    }

    /**
     * Rotate the robot to desired angle position using the built in gyro in IMU sensor onboard the REV expansion hub
     * The angle is specified relative to the start position of the robot when the IMU is initialized
     * and gyro angle is intialized to zero degrees.
     *
     * @param targetAngle   The desired target angle position in degrees. Positive value is counter clockwise from initial zero position
     *                      Negative value is clock wise from initial zero. Angle value wraps around at 180 and -180 degrees.
     * @param turnSpeed     The speed at which to drive the motors for the rotation. 0.0 < turnSpeed <= 1.0
     */
    public void gyroRotateToHeading(double targetAngle, double turnSpeed, double timeout) {

        // determine current angle of the robot
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double robotAngle = angles.firstAngle;
        double delta = MathFunctions.angleWrap(targetAngle - robotAngle);
        double direction = Math.signum(delta); // positive angle requires CCW rotation, negative angle requires CW
        turnSpeed = Math.abs(turnSpeed);

        movementStatus = String.format(Locale.US,"Rot Tgt=%.2f | Spd=%1.2f | TO=%1.2f", targetAngle, turnSpeed, timeout);
        ElapsedTime runtime = new ElapsedTime();
        // while the robot heading has not reached the targetAngle (delta sign flips), and consider reached within a margin of 0.2 degrees
        // Margin is used because the overshoot for even small gyro rotation is 0.05 to 0.15 degrees
        while (myOpMode.opModeIsActive() && ((direction * delta) > 0.2) && (runtime.seconds() < timeout)) {

            // slow down linearly for the last N degrees rotation remaining, ROTATE_SPEED_MIN is required to overcome inertia
            double speed = Range.clip(turnSpeed*Math.abs(delta)/30, ROTATE_SPEED_MIN, turnSpeed);
            // the sign of delta determines the direction of rotation of robot
            robot.driveTank(0, direction * speed);
            myOpMode.sleep(50); // allow some time for the motors to actuate
            robotAngle = robot.imu.getAngularOrientation().firstAngle;
            delta = MathFunctions.angleWrap(targetAngle - robotAngle);

            myOpMode.telemetry.addLine("Gyro Rot ")
                    .addData("Tgt", "%.2f", targetAngle)
                    .addData("Act", "%.2f", robotAngle)
                    .addData("Dlt", "%.2f", delta)
                    .addData("Spd", "%.2f", speed);
            myOpMode.telemetry.update();
        }
        robot.stopDriving();
        movementStatus = String.format(Locale.US,"Rot Tgt=%.2f | Act=%.2f | Spd=%1.2f | t=%1.2f", targetAngle, robotAngle, turnSpeed, runtime.seconds());
    }

    /**
     * Rotate the robot to desired angle position using the odometry position feedback
     *
     * @param targetAngle   The desired target angle position in degrees
     * @param turnSpeed     The speed at which to drive the motors for the rotation. 0.0 < turnSpeed <= 1.0
     */
    public void odometryRotateToHeading(double targetAngle, double turnSpeed, double timeout) {

        // determine current angle of the robot
        double robotAngle = globalPosition.getOrientationDegrees();
        double delta = MathFunctions.angleWrap(targetAngle - robotAngle);
        double direction = Math.signum(delta); // positive angle requires CCW rotation, negative angle requires CW
        turnSpeed = Math.abs(turnSpeed);

        movementStatus = String.format(Locale.US,"Rot Tgt=%.2f | Spd=%1.2f | TO=%1.2f", targetAngle, turnSpeed, timeout);
        ElapsedTime runtime = new ElapsedTime();
        // while the robot heading has not reached the targetAngle (delta sign flips), and consider reached within a margin of 0.2 degrees
        // Margin is used because the overshoot for even small gyro rotation is 0.05 to 0.15 degrees
        while (myOpMode.opModeIsActive() && ((direction * delta) > 0.2) && (runtime.seconds() < timeout)) {

            // slow down linearly for the last N degrees rotation remaining, ROTATE_SPEED_MIN is required to overcome inertia
            double speed = Range.clip(turnSpeed*Math.abs(delta)/30, ROTATE_SPEED_MIN, turnSpeed);
            // the sign of delta determines the direction of rotation of robot
            robot.driveTank(0, direction * speed);
            myOpMode.sleep(50); // allow some time for the motors to actuate
            robotAngle = globalPosition.getOrientationDegrees();
            delta = MathFunctions.angleWrap(targetAngle - robotAngle);

            myOpMode.telemetry.addLine("Odom Rot ")
                    .addData("Tgt", "%.2f", targetAngle)
                    .addData("Act", "%.2f", robotAngle)
                    .addData("Dlt", "%.2f", delta)
                    .addData("Spd", "%.2f", speed);
            myOpMode.telemetry.update();
        }
        robot.stopDriving();
        movementStatus = String.format(Locale.US,"Rot Tgt=%.2f | Act=%.2f | Spd=%1.2f | t=%1.2f", targetAngle, robotAngle, turnSpeed, runtime.seconds());
    }

    public void rotateToHeading(double targetAngle, double turnSpeed, double timeout) {
        gyroRotateToHeading(targetAngle, turnSpeed, timeout);
    }

    public void rotateToHeading(double targetAngle, double turnSpeed) {
        gyroRotateToHeading(targetAngle, turnSpeed, TIMEOUT_ROTATE);
    }

    public void rotateToHeading(double targetAngle) {
        gyroRotateToHeading(targetAngle, ROTATE_SPEED_DEFAULT, TIMEOUT_ROTATE);
    }

    /**
     * Drive the robot at specified speed towards the specified target position on the field.
     * The current position of the robot obtained using odometry readings.
     *
     * @param x     Target position global x coordinate value (inches)
     * @param y     Target position global y coordinate value (inches)
     * @param speed Speed/Power used to drive. Must be in range of -1.0 <= speed <= 1.0a
     * @param timeout Time (seconds) to complete the move or abort
     * @param slowDownAtEnd true if robot should slow down close to destination, to avoid overshooting
     */
    public void goToPosition(double x, double y, double speed, double timeout, boolean slowDownAtEnd) {

        ElapsedTime runtime = new ElapsedTime();
        double distance = 200; // greater than the diagonal length of the FTC field
        double previous;

        movementStatus = String.format(Locale.US,"GoToPos X=%3.2f, Y=%2.2f, Spd=%1.1f, TO=%1.1f", x, y, speed, timeout);
        speed = Range.clip(speed, DRIVE_SPEED_MIN, DRIVE_SPEED_MAX);
        runtime.reset();
        // we consider reaching the destination (x,y) position if less than DIST_MARGIN inches away OR
        // if the distance from the destination starts increases from its previous value
        while (myOpMode.opModeIsActive() && (runtime.seconds() < timeout) && (distance >= DIST_MARGIN)) {
            previous = distance;
            distance = goTowardsPosition(x, y, speed, slowDownAtEnd);
            // Avoid oscillations at the end, near the target destination (DIST_NEAR threshold)
            // If the distance from the destination starts increasing from its previous value,
            // then robot may have overshot the target coordinate location, stop there
            if  (distance < DIST_NEAR && distance > previous) {
                break;
            }
        }
        robot.stopDriving();
        movementStatus = String.format(Locale.US,"Reached X=%3.2f, Y=%2.2f, Spd=%1.1f in T=%1.1f", x, y, speed, runtime.seconds());
    }

    public void goToPosition(double x, double y, boolean slowDownAtEnd) {

        goToPosition(x, y, DRIVE_SPEED_DEFAULT, TIMEOUT_DEFAULT, slowDownAtEnd);
    }

    public void goToPosition(double x, double y, double speed, double timeout) {

        goToPosition(x, y, speed, timeout, true);
    }

    public void goToPosition(double x, double y) {

        goToPosition(x, y, DRIVE_SPEED_DEFAULT, TIMEOUT_DEFAULT);
    }

    /**
     * Drive the robot at specified speed towards the specified target position on the field.
     * Note that we are not waiting to reach the target position. This method only sets wheel power
     * in the direction of the target position and must be called again repeatedly at an update
     * interval, typically 50ms - 75ms.
     * The current position of the robot obtained using odometry readings and wheel power are both
     * calculated again at each call to this method.
     *
     * @param x     Target position global x coordinate value (inches)
     * @param y     Target position global y coordinate value (inches)
     * @param speed Speed/Power used to drive. Must be in range of -1.0 <= speed <= 1.0
     * @return <em>true</em> if sucessfully issued command to robot to drive, <em>false</em> if reached the destination
     */
    public double goTowardsPosition(double x, double y, double speed, boolean slowDownAtEnd) {

        double distanceToPosition = Math.hypot(globalPosition.getXinches() - x,  globalPosition.getYinches() - y);
        double absoluteAngleToPosition = Math.atan2(y - globalPosition.getYinches(), x - globalPosition.getXinches());
        double relativeAngleToPosition;

        if (robot.isDirectionForward()) {
            relativeAngleToPosition = MathFunctions.angleWrapRad(absoluteAngleToPosition - globalPosition.getOrientationRadians());
        }
        else { // (robot.isFrontLiftarm())
        // override in case of Robot front face has been REVERSED. The motors will run swapped (left front runs as right back)
        // The only control we need to change is to calculate the turn power for driving in reverse direction
        // This is done by adding 180 degrees (or PI radians) to the relative Angle (or to the robot orientation angle)
            relativeAngleToPosition = MathFunctions.angleWrapRad(absoluteAngleToPosition - globalPosition.getOrientationRadians() + Math.PI);
        }

        double drivePower = speed;
        // when within DIST_SLOWDOWN inches of target, reduce the speed proportional to remaining distance to target
        if (slowDownAtEnd && distanceToPosition < DIST_SLOWDOWN) {
            // however absolute minimum power is required otherwise the robot cannot move the last couple of inches

            // enable this code for delayed aggressive braking - overshoots target but saves some time
            //double slowPower = Range.clip(distanceToPosition / DIST_SLOWDOWN, DRIVE_SPEED_MIN, DRIVE_SPEED_MAX);
            //drivePower = Math.min(speed, slowPower);

            // enable this code instead for more gentle slow down at end - stop closer to target position
            double slowPower = (distanceToPosition / DIST_SLOWDOWN) * speed;
            drivePower = Math.max(DRIVE_SPEED_MIN, slowPower);
        }
        // set turnspeed proportional to the amount of turn required, however beyond 30 degrees turn, full speed is ok
        // note here that positive angle means turn left (since angle is measured counter clockwise from X-axis)
        // this must match the behavior of Mecabot.DriveTank() method used below.
        double turnPower = Range.clip(relativeAngleToPosition / Math.toRadians(30), -1.0, 1.0) * drivePower;
        // no minimum for turnPower until robot auto driving tests indicate a need.

        myOpMode.telemetry.addLine("GoToPos ").addData("Dist", " %4.2f in", distanceToPosition).addData("Rel Angle", " %4.2f deg", Math.toDegrees(relativeAngleToPosition));
        myOpMode.telemetry.addLine("Power ").addData("drive", " %.2f", drivePower).addData("turn", "%.2f", turnPower);
        myOpMode.telemetry.update();

        // let's stop driving when within a short distance of the destination. This threshold may need to be tuned.
        // A threshold is necessary to avoid oscillations caused by overshooting of target position.
        // This check could be done early in the method, however it is done towards end deliberately to get telemetry readouts
        if (distanceToPosition < DIST_MARGIN) {
            robot.stopDriving();
        }
        else {
            robot.driveTank(drivePower, turnPower);
        }

        return distanceToPosition;
    }

    /**
     * Move to a position at specified X-coordinate, while maintaining the current Y-coordinate value
     *
     * @param targetX   the target X coordinate position
     * @param speed     driving speed for movement
     */
    public void goToXPosition(double targetX, double speed, double timeout) {

        double curX = globalPosition.getXinches();

        // do not move less than 1 inch, that is our margin threshold for reaching the target coordinate.
        if (Math.abs(targetX - curX) < DIST_MARGIN) {
            return;
        }

        // THIS CODE IS INCOMPATIBLE WITH DRIVING THE ROBOT IN REVERSE DIRECTION
//        // First lets point heading in the direction of movement, so we can drive straight
//        if (targetX - curX > 0) { // if we need to move towards positive X-Axis
//            gyroRotateToHeading(0.0, ROTATE_SPEED_SLOW);
//        }
//        else { // we need to move towareds negative X-Axis
//            gyroRotateToHeading(180.0, ROTATE_SPEED_SLOW);
//        }

        // the gyro rotation moves the robot X,Y position, we will ignore that small amount
        // get the Y coorindate now (after gyroRotate()) so we drive stright only along X-axis
        double curY = globalPosition.getYinches();

        // Now lets go to the target destination coordinate
        goToPosition(targetX, curY, speed, timeout);
    }

    public void goToXPosition(double targetX) {
        goToXPosition(targetX, DRIVE_SPEED_DEFAULT, TIMEOUT_DEFAULT);
    }

    /**
     * Move to a position at specified Y-coordinate, while maintaining the current X-coordinate value
     *
     * @param targetY   the target Y coordinate position
     * @param speed     driving speed for movement
     */
    public void goToYPosition(double targetY, double speed, double timeout) {

        double curY = globalPosition.getYinches();

        // do not move less than 1 inch, that is our margin threshold for reaching the target coordinate.
        if (Math.abs(targetY - curY) < DIST_MARGIN) {
            return;
        }

        // THIS CODE IS INCOMPATIBLE WITH DRIVING THE ROBOT IN REVERSE DIRECTION
//        // First lets point heading in the direction of movement, so we can drive straight
//        if (targetY - curY > 0) { // if we need to move towards positive Y-Axis
//            gyroRotateToHeading(90.0, ROTATE_SPEED_SLOW);
//        }
//        else { // we need to move towareds negative Y-Axis (which is off the field, so never reach here)
//            gyroRotateToHeading(-90.0, ROTATE_SPEED_SLOW);
//        }

        // the gyro rotation moves the robot X,Y position, we will ignore that small amount
        // get the X coorindate now (after gyroRotate()) so we drive stright only along X-axis
        double curX = globalPosition.getXinches();

        // Now lets go to the target destination coordinate
        goToPosition(curX, targetY, speed, timeout);
    }

    public void goToYPosition(double targetY) {
        goToYPosition(targetY, DRIVE_SPEED_DEFAULT, TIMEOUT_DEFAULT);
    }

    /**
     * Move the specified distance (in inches), either normal or mecanum sideways movement.
     * The movement direction is controlled by the sign of the first parameter, distance in inches to move
     * This method uses odometry feedback to determine Robot current position during movement
     * Move FORWARD : inches +ve value, mecanumSideways = false;
     * Move REVERSE : inches -ve value, mecanumSideways = false;
     * Move RIGHT   : inches +ve value, mecanumSideways = true;
     * Move LEFT    : inches -ve value, mecanumSideways = true;
     * @param inches            distance to move
     * @param driveType         driving method (tank, mecanum, and diagonal)
     * @param speed             driving speed for the movement
     * @param timeout           Time (seconds) to complete the move or abort
     */
    public void odometryMoveDistance(double inches, DriveType driveType, double speed, double timeout) {
        // do not move less than 1 inch, that is our margin threshold for reaching the target coordinate.
        if (Math.abs(inches) < DIST_MARGIN) {
            return;
        }
        robot.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // if distance is negative, direction must be reversed, both forwards/backwards or left/right side
        // which is done by negative power to the motors
        if (inches < 0) {
            speed = -speed;
        }
        double origX = globalPosition.getXinches();
        double origY = globalPosition.getYinches();
        double curX;
        double curY;
        double distance = 0;
        ElapsedTime runtime = new ElapsedTime();

        movementStatus = String.format(Locale.US,"Dist %.1f in, from (%.1f, %.1f) S=%1.1f TO=%1.1f", inches, origX, origY, speed, timeout);

        runtime.reset();
        while (myOpMode.opModeIsActive() && (runtime.seconds() < timeout) && (distance < Math.abs(inches))) {
            switch (driveType) {
                case MECANUM:
                    robot.driveMecanum(speed);
                    break;
                case DIAGONAL:
                    robot.driveDiagonal(speed);
                    break;
                case TANK:
                    robot.driveTank(speed, 0.0);
                    break;
            }
            curX = globalPosition.getXinches();
            curY = globalPosition.getYinches();
            distance = Math.hypot((curX - origX), (curY - origY));

            myOpMode.telemetry.addLine("MoveDist ").addData("now at", "%.1f of %.1f in", distance, inches);
            myOpMode.telemetry.update();
        }
        // Reached within threshold of target distance.
        robot.stopDriving();
        movementStatus = String.format(Locale.US,"Done D=%.1f in, from (%.1f, %.1f) S=%1.1f in T=%1.1f", distance, origX, origY, speed, runtime.seconds());

    }

    public void odometryMoveDistance(double inches, DriveType driveType, double speed) {
        odometryMoveDistance(inches, driveType, speed, TIMEOUT_DEFAULT);
    }

    /**
     * Move the specified distance (in inches), either normal or mecanum sideways movement.
     * The movement direction is controlled by the sign of the first parameter, distance in inches to move
     * This method uses odometry feedback to determine Robot current position during movement
     * Move FORWARD : inches +ve value, mecanumSideways = false;
     * Move REVERSE : inches -ve value, mecanumSideways = false;
     * Move RIGHT   : inches +ve value, mecanumSideways = true;
     * Move LEFT    : inches -ve value, mecanumSideways = true;
     * @param inches            distance to move
     * @param driveType         driving method (tank, mecanum, and diagonal)
     */
    public void odometryMoveDistance(double inches, DriveType driveType) {
        odometryMoveDistance(inches, driveType, DRIVE_SPEED_DEFAULT, TIMEOUT_DEFAULT);
    }

    /**
     * Move robot forward or backward, +ve distance moves forward, -ve distance moves backward
     * @param inches distance to move
     */
    public void odometryMoveForwardBack(double inches) {
        odometryMoveDistance(inches, DriveType.TANK, DRIVE_SPEED_DEFAULT);
    }

    /**
     * Move robot forward or backward, +ve distance moves forward, -ve distance moves backward
     * @param inches distance to move
     * @param speed  speed of movement
     */
    public void odometryMoveForwardBack(double inches, double speed) {
        odometryMoveDistance(inches, DriveType.TANK, speed, TIMEOUT_DEFAULT);
    }

    /**
     * Move robot left or right, +ve distance moves right, -ve distance moves left
     * @param inches distance to move
     */
    public void odometryMoveRightLeft(double inches) {
        odometryMoveDistance(inches, DriveType.MECANUM);
    }

    /**
     * Move robot left or right, +ve distance moves right, -ve distance moves left
     * @param inches distance to move
     * @param speed  speed of movement
     */
    public void odometryMoveRightLeft(double inches, double speed) {
        odometryMoveDistance(inches, DriveType.MECANUM, speed, TIMEOUT_DEFAULT);
    }

    /*
     * Move robot forward or backward, +ve distance moves forward, -ve distance moves backward
     */
    public void encoderMoveForwardBack(double inches) {
        encoderMoveDistance( inches, false, DEFAULT_SPEED);
    }

    public void encoderMoveForwardBack(double inches, double speed) {
        encoderMoveDistance( inches, false, speed);
    }

   /*
    * Move robot left or right, +ve distance moves right, -ve distance moves left
    */
    public void encoderMoveRightLeft(double inches) {
        encoderMoveDistance(inches, true, DEFAULT_SPEED);
    }

    public void encoderMoveRightLeft(double inches, double speed) {
        encoderMoveDistance(inches, true, speed);
    }

    /*
     * Move robot left or right, +ve distance moves LEFT, -ve distance moves RIGHT
     */
    @Deprecated
    public void encoderMoveLeftRight(double inches) {
        encoderMoveDistance(inches * -1.0, true, DEFAULT_SPEED);
    }

    @Deprecated
    public void encoderMoveLeftRight(double inches, double speed) {
        encoderMoveDistance(inches * -1.0, true, speed);
    }

    /**
     * Move the specified distance (in inches), either normal or mecanum sideways movement.
     * The movement direction is controlled by the sign of the first parameter, distance in inches to move
     * This method uses the wheel encoders to move exactly the desired distance.
     * @param inches            distance to move
     * @param mecanumSideways   Mecanum sideways movement if true, Normal tank movement if false
     * @param speed             driving speed for the movement
     */

    private void encoderMoveDistance(double inches, boolean mecanumSideways, double speed) {

        //convert inches to tick counts
        int driverEncoderTarget = (int) (ENCODER_TICKS_PER_INCH * inches);

        // default is drive straight all wheels drive same direction (forward or backward depending on sign)
        int leftFront = driverEncoderTarget;
        int leftBack = driverEncoderTarget;
        int rightFront = driverEncoderTarget;
        int rightBack = driverEncoderTarget;

        // for mecanum sideways movement, move Right when goForwardOrRight is true
        // Right wheels move inside, Left wheels move outside
        if (mecanumSideways) {
            rightFront = -driverEncoderTarget;  // drive backward for inside
            rightBack = driverEncoderTarget;    // drive forward for inside
            leftFront = driverEncoderTarget;    // drive forward for outside
            leftBack = -driverEncoderTarget;    // drive backward for outside
        }
        // same code above works for mecanum move Left also. False value of goForwardOrRight already flipped the sign above

        // set target position for encoder Drive
        robot.resetDriveEncoder();
        robot.setTargetPosition(leftFront, leftBack, rightFront, rightBack);

        // Set the motors to run to the necessary target position
        robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the power of the motors to whatever speed is needed
        robot.setDrivePower(speed);

        myOpMode.telemetry.addLine("Driving inches | ").addData("outer", inches).addData("inner", inches);

        // loop until motors are busy driving, update current position on driver station using telemetry
        waitToReachTargetPosition(WheelPosition.RIGHT_FRONT, leftFront, leftBack, rightFront, rightBack);

    }

    // Rotate around Robot own center
    public void encoderRotate(double inches, boolean counterClockwise, double speed) {

        // Green intake wheels is front of robot,
        // counterClockwise means Right wheels turning forward, Left wheels turning backwards

        int ticks = (int) (ENCODER_TICKS_PER_INCH * inches);

        int leftFront = counterClockwise ? -ticks : +ticks;
        int leftBack = leftFront;
        int rightFront = counterClockwise ? +ticks : -ticks;
        int rightBack = rightFront;

        // set target position for encoder Drive
        robot.resetDriveEncoder();
        robot.setTargetPosition(leftFront, leftBack, rightFront, rightBack);

        // Set the motors to run to the necessary target position
        robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setDrivePower(speed);

        myOpMode.telemetry.addLine("Driving inches | ").addData("outer", inches).addData("inner", inches);

        // loop until motors are busy driving, update current position on driver station using telemetry
        waitToReachTargetPosition(counterClockwise ? WheelPosition.RIGHT_FRONT : WheelPosition.LEFT_FRONT,
                leftFront, leftBack, rightFront, rightBack);
    }

    public void encoderRotate(double inches, boolean counterClockwise) {
        encoderRotate(inches, counterClockwise, DEFAULT_SPEED);
    }

    // Turn in an arc
    // Assume robot is touching the outer edge of a tile and we want to rotate it in a circular arc
    // This is not rotating around its center axis but an arc with center of circle outside the robot where the tiles meet
    // Outer wheels run along outer circle and inner wheels run along an inner circle

    public void encoderTurn(double inches, boolean counterClockwise, double speed) {

        // Green intake wheels is front of robot, counterClockwise means Right wheels on outer circle
        double outerWheelInches = inches * 1;
        double innerWheelInches = inches / OUTER_TO_INNER_TURN_SPEED_RATIO;

        int outerWheelTicks = (int) (ENCODER_TICKS_PER_INCH * outerWheelInches);
        int innerWheelTicks = (int) (ENCODER_TICKS_PER_INCH * innerWheelInches);

        int leftFront = counterClockwise ? innerWheelTicks : outerWheelTicks;
        int leftBack = leftFront;
        int rightFront = counterClockwise ? outerWheelTicks : innerWheelTicks;
        int rightBack = rightFront;

        // set target position for encoder Drive
        robot.resetDriveEncoder();
        robot.setTargetPosition(leftFront, leftBack, rightFront, rightBack);

        // Set the motors to run to the necessary target position
        robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        double wheelPower = Range.clip(speed, 0.0, 1.0);
        double insideWheelPower = wheelPower / OUTER_TO_INNER_TURN_SPEED_RATIO;
        if (counterClockwise) {
            robot.setDrivePower(insideWheelPower, wheelPower);
        } else {
            robot.setDrivePower(wheelPower, insideWheelPower);
        }

        myOpMode.telemetry.addLine("Driving inches | ").addData("outer", outerWheelInches).addData("inner", innerWheelInches);

        // loop until motors are busy driving, update current position on driver station using telemetry
        waitToReachTargetPosition(counterClockwise ? WheelPosition.RIGHT_FRONT : WheelPosition.LEFT_FRONT,
                leftFront, leftBack, rightFront, rightBack);
    }

    public void encoderTurn(double inches, boolean counterClockwise) {
        encoderTurn(inches, counterClockwise, DEFAULT_SPEED);
    }

    private void waitToReachTargetPosition(WheelPosition dominantWheel, int leftFront, int leftBack, int rightFront, int rightBack) {

        // Loop until motors are no longer busy.
        while (robot.leftFrontDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightBackDrive.isBusy()) {

            myOpMode.telemetry.addLine("Target position | ").addData("LF", leftFront).addData("RF", rightFront);
            myOpMode.telemetry.addLine("Target position | ").addData("LB", leftBack).addData("RB", rightBack);
            myOpMode.telemetry.addLine("Current position | ").addData("LF", robot.leftFrontDrive.getCurrentPosition()).addData("RF", robot.rightFrontDrive.getCurrentPosition());
            myOpMode.telemetry.addLine("Current position | ").addData("LB", robot.leftBackDrive.getCurrentPosition()).addData("RB", robot.rightBackDrive.getCurrentPosition());
            myOpMode.telemetry.update();

            //encoder reading a bit off target can keep us in this loop forever, so given an error margin here
            if ((dominantWheel == WheelPosition.LEFT_FRONT) && Math.abs(robot.leftFrontDrive.getCurrentPosition() - leftFront) < ENCODER_TICKS_ERR_MARGIN) {
                break;
            }
            else if ((dominantWheel == WheelPosition.LEFT_BACK) && Math.abs(robot.leftBackDrive.getCurrentPosition() - leftBack) < ENCODER_TICKS_ERR_MARGIN) {
                break;
            }
            else if ((dominantWheel == WheelPosition.RIGHT_FRONT) && Math.abs(robot.rightFrontDrive.getCurrentPosition() - rightFront) < ENCODER_TICKS_ERR_MARGIN) {
                break;
            }
            else if ((dominantWheel == WheelPosition.RIGHT_BACK) && Math.abs(robot.rightBackDrive.getCurrentPosition() - rightBack) < ENCODER_TICKS_ERR_MARGIN) {
                break;
            }
        }

        // Stop powering the motors - robot has moved to intended position
        robot.stopDriving();
        myOpMode.telemetry.addData("DONE driving LF position=",robot.leftFrontDrive.getCurrentPosition());
        // Turn off RUN_TO_POSITION
        robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        myOpMode.sleep(250);

    }

    /*
     * Utility functions for use by sub classes
     * Useful for subsystem motors other than the drive train
     */
    public void waitUntilMotorBusy(DcMotor motor) {
        ElapsedTime runTime = new ElapsedTime();
        while (motor.isBusy() && myOpMode.opModeIsActive() && (runTime.seconds() < TIMEOUT_SHORT)){
            myOpMode.sleep(50);
        }
    }
    public void motorRunToPosition(DcMotor motor, int position, double speed) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);
        waitUntilMotorBusy(motor);
        motor.setPower(MOTOR_STOP_SPEED);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}