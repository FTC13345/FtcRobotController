package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal;

public class MecabotDriver implements Runnable {

    static final double TURN_FACTOR = 0.6;    // slow down turning speed

    // member variables for state
    protected LinearOpMode  myOpMode;       // Access to the OpMode object
    protected MecabotMove   robot;          // Access to the Robot hardware
    protected Gamepad       gamepad1;
    protected Telemetry     telemetry;

    protected Thread        myThread;
    protected boolean       isRunning = false;
    protected double        speedMultiplier = MecabotMove.DRIVE_SPEED_MAX;

    // record position that we need to return to repeatedly
    double xpos, ypos, tpos;
    boolean autoDriving = false;

    /* Constructor */
    public MecabotDriver(LinearOpMode opMode, MecabotMove aRobot) {
        // Save reference to OpMode and Hardware map
        myOpMode = opMode;
        gamepad1 = opMode.gamepad1;
        telemetry = opMode.telemetry;
        robot = aRobot;
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

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning && myOpMode.opModeIsActive()) {
            setup();
            buttonHandler();
            driveMecabot();
            autodrive();
            telemetry.update();
            myOpMode.idle();
        }
    }

    public void setup() {
        if (gamepad1.start) {
            // Toggle which face of the Robot is front for driving
            if (gamepad1.right_bumper) {
                robot.setDirectionForward();
            } else if (gamepad1.left_bumper) {
                robot.setDirectionReverse();
            }
        }
        else { // !gamepad1.start --> which means bumper buttons pressed alone
            //update speedMultiplier for FAST or SLOW driving
            if (gamepad1.right_bumper) {
                speedMultiplier = MecabotMove.DRIVE_SPEED_MAX;
                robot.setFastBlue();
                // as a dual action of this button stop autodriving
                autoDriving = false;
            }
            else if (gamepad1.left_bumper) {
                speedMultiplier = MecabotMove.DRIVE_SPEED_DEFAULT;
                robot.setSlowBlue();
                // as a dual action of this button stop autodriving
                autoDriving = false;
            }
        }
    }

    public void buttonHandler() {
        if (gamepad1.a) {
            robot.odometryRotateToHeading(FieldUGoal.ANGLE_POS_X_AXIS, MecabotMove.ROTATE_SPEED_DEFAULT, MecabotMove.TIMEOUT_ROTATE);
        }
        else if (gamepad1.b) {
            robot.gyroRotateToHeading(FieldUGoal.ANGLE_POS_X_AXIS, MecabotMove.ROTATE_SPEED_DEFAULT, MecabotMove.TIMEOUT_ROTATE);
        }
    }

    public void driveMecabot() {
        // if joystick is inactive brakes will be applied during autodrive() therefore don't go in
        if (autoDriving) {
            return;
        }

        double power, turn;

        // square the joystick values to change from linear to logarithmic scale
        // this allows more movement of joystick for less movement of robot, thus more precision at lower speeds
        // at the expense of some loss of precision at higher speeds, where it is not required.

        // when we want to move sideways (MECANUM)
        if (gamepad1.left_trigger > 0) {
            power = -gamepad1.left_trigger; // negative power to move LEFT
            // Square the number but retain the sign to convert to logarithmic scale
            // scale the range to 0.30 <= abs(power) <= 1.0 and preserve the sign
            power = Math.signum(power) * (0.25 + (0.75 * power * power)) * speedMultiplier;
            robot.driveMecanum(power);
            telemetry.addData("Mecanum Left ", "%.2f", power);
        }
        else if (gamepad1.right_trigger > 0) {
            power = gamepad1.right_trigger; // positive power to move RIGHT
            // Square the number but retain the sign to convert to logarithmic scale
            // scale the range to 0.30 <= abs(power) <= 1.0 and preserve the sign
            power = Math.signum(power) * (0.25 + (0.75 * power * power)) * speedMultiplier;
            robot.driveMecanum(power);
            telemetry.addData("Mecanum Right  ", "%.2f", power);
        }
        // normal tank movement
        else {  // when joystick is inactive, this applies brakes, be careful to avoid during autodrive
            // forward press on joystick is negative, backward press (towards human) is positive
            // right press on joystick is positive value, left press is negative value
            // reverse sign of joystick values to match the expected sign in driveTank() method.
            power = -gamepad1.left_stick_y;
            turn = -gamepad1.right_stick_x;

            // Square the number but retain the sign to convert to logarithmic scale
            // scale the range to 0.25 <= abs(power) <= 1.0 and preserve the sign
            power = Math.signum(power) * (0.2 + (0.8 * power * power));
            // OR
            // use this to scale the range without squaring the power value
            //power = Math.signum(power) * (0.25 + (0.75 * Math.abs(power)));
            // OR
            // use this to square the power while preserving the sign, without scaling the range
            //power *= Math.abs(power);

            // similarly for turn power, except also slow down by TURN_FACTOR
            turn = Math.signum(turn) * (0.1 + (TURN_FACTOR * turn * turn));

            robot.driveTank(power, turn);
            telemetry.addData("Tank Power", "Drive=%.2f Turn=%.2f", power, turn);
        }
    }

    public void autodrive() {
/*
        // This block of code was used to record the position of robot on the field and
        // then later auto drive back to same location. Used in Skystone, no longer used in Ultimate Goal.
       if ((gamepad1.x) && (gamepad1.start)) {
            xpos = globalPosition.getXinches();
            ypos = globalPosition.getYinches();
            tpos = globalPosition.getOrientationDegrees();
            telemetry.addData("Locked Position", "X %2.2f | Y %2.2f | Angle %3.2f", xpos, ypos, tpos);
        }
        if ((gamepad1.y) && (gamepad1.start)){
            robot.setDirectionReverse();
            autoDriving = true;
        }
 */

        if (autoDriving) {
            telemetry.addData("Driving Towards", "X %2.2f | Y %2.2f | Angle %3.2f", xpos, ypos, tpos);
            double distance = robot.goTowardsPosition(xpos, ypos, MecabotMove.DRIVE_SPEED_DEFAULT, true);
            if (distance < MecabotMove.DIST_MARGIN) { // we have reached
                autoDriving = false;
            }
        }
    }
}
