package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.*;

public class TeleOpDriver implements Runnable {

    static final double TURN_FACTOR = 0.6;    // slow down turning speed

    // member variables for state
    protected LinearOpMode  myOpMode;       // Access to the OpMode object
    protected RRMecanumDrive drive;
    protected MecabotDrive robot;          // Access to the Robot hardware
    protected Gamepad       gamepad1;
    protected Telemetry     telemetry;

    protected Thread        myThread;
    protected boolean       isRunning = false;
    protected double        speedMultiplier = MecabotDrive.DRIVE_SPEED_MAX;

    // record position that we need to return to repeatedly
    double xpos, ypos, tpos;
    boolean autoDriving = false;

    /* Constructor */
    public TeleOpDriver(LinearOpMode opMode, RRMecanumDrive rrmdrive, MecabotDrive aRobot) {
        // Save reference to OpMode and Hardware map
        myOpMode = opMode;
        gamepad1 = opMode.gamepad1;
        telemetry = opMode.telemetry;
        drive = rrmdrive;
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
            driveTeleOp();
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
                speedMultiplier = MecabotDrive.DRIVE_SPEED_MAX;
                robot.setFastBlue();
                // as a dual action of this button stop autodriving
                autoDriving = false;
            }
            else if (gamepad1.left_bumper) {
                speedMultiplier = MecabotDrive.DRIVE_SPEED_DEFAULT;
                robot.setSlowBlue();
                // as a dual action of this button stop autodriving
                autoDriving = false;
            }
        }
    }

    public void buttonHandler() {
        if (gamepad1.a) {
            robot.odometryRotateToHeading(ANGLE_POS_X_AXIS, MecabotDrive.ROTATE_SPEED_DEFAULT, MecabotDrive.TIMEOUT_ROTATE);
        }
        else if (gamepad1.b) {
            robot.gyroRotateToHeading(ANGLE_POS_X_AXIS, MecabotDrive.ROTATE_SPEED_DEFAULT, MecabotDrive.TIMEOUT_ROTATE);
        }
    }

    public void driveTeleOp() {
        // if joystick is inactive brakes will be applied during autodrive() therefore don't go in
        if (autoDriving) {
            return;
        }

        // square the joystick values to change from linear to logarithmic scale
        // this allows more movement of joystick for less movement of robot, thus more precision at lower speeds
        // at the expense of some loss of precision at higher speeds, where it is not required.
        double power = -gamepad1.left_stick_y;
        // Square the number but retain the sign to convert to logarithmic scale
        // scale the range to 0.2 <= abs(power) <= 1.0 and preserve the sign
        power = Math.signum(power) * (0.2 + (0.8 * power * power)) * speedMultiplier;
        // OR
        // use this to scale the range without squaring the power value
        //power = Math.signum(power) * (0.25 + (0.75 * Math.abs(power)));
        // OR
        // use this to square the power while preserving the sign, without scaling the range
        //power *= Math.abs(power);

        double strafe = -gamepad1.left_stick_x;
        strafe = Math.signum(strafe) * (0.2 + (0.8 * strafe * strafe)) * speedMultiplier;

        // similarly for turn power, except also slow down by TURN_FACTOR
        double turn = -gamepad1.right_stick_x;
        turn = Math.signum(turn) * (0.1 + (TURN_FACTOR * turn * turn));

        if (drive != null) {
            driveRRM(power, strafe, turn);
        } else {
            driveMecabot(power, strafe, turn);
        }
    }

    public void driveRRM(double power, double strafe, double turn) {

        drive.setWeightedDrivePower(
                new Pose2d(power, strafe, turn)
        );

        // Update the drive class
        drive.update();

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();
        // Print pose to telemetry
        telemetry.addLine("Runner Position ")
                .addData("X", "%2.2f", poseEstimate.getX())
                .addData("Y", "%2.2f", poseEstimate.getY())
                .addData("Angle", "%3.2f", Math.toDegrees(poseEstimate.getHeading()));
        telemetry.update();
    }


    public void driveMecabot(double power, double strafe, double turn) {
        // when we want to move sideways (MECANUM)
        if (Math.abs(strafe) > 0.2) {
            robot.driveMecanum(strafe);
            telemetry.addData("Mecanum ", "%.2f", power);
        }
        // normal tank movement
        else {  // when joystick is inactive, this applies brakes, be careful to avoid during autodrive
            // forward press on joystick is negative, backward press (towards human) is positive
            // right press on joystick is positive value, left press is negative value
            // reverse sign of joystick values to match the expected sign in driveTank() method.

            robot.driveTank(power, turn);
            //telemetry.addData("Tank Power", "Drive=%.2f Turn=%.2f", power, turn);
        }
    }

    public void autodrive() {

        if (autoDriving) {
            if (drive.isBusy()) {
                drive.update();
            }
            else {
                autoDriving = false;
            }
            return;
        }
        if (gamepad1.x) {
            Trajectory goToShoot = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(ORIGIN - 4.0, flip4Red(GOALY - ROBOT_RADIUS), ANGLE_POS_X_AXIS))
                    .build();
            drive.followTrajectoryAsync(goToShoot);
            autoDriving = true;
        }
/*
        if (autoDriving) {
            telemetry.addData("Driving Towards", "X %2.2f | Y %2.2f | Angle %3.2f", xpos, ypos, tpos);
            double distance = robot.goTowardsPosition(xpos, ypos, MecabotDrive.DRIVE_SPEED_DEFAULT, true);
            if (distance < MecabotDrive.DIST_MARGIN) { // we have reached
                autoDriving = false;
            }
        }
 */
    }
}
