package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.Mecabot;

import java.util.Locale;

/**
 * Rotate the robot using heading value from the Gyroscope included in the IMU inside REV Control Hub or Expansion Hub.
 * This OpMode uses PIDF feedback controller for increasing the accuracy and minimize overshoot
 * The PIDF coefficients need to be determined using a separate tuner program.
 *
 */
@TeleOp(name = "Gyro Rotate Demo", group = "Test")
//@Disabled                            // Comment this out to add to the opmode list
public class GyroRotate extends LinearOpMode {

    public static PIDFCoefficients GYRO_ROTATE_PID = new PIDFCoefficients(0, 0, 0, 0);
    public static double TARGET_HEADING = 0.0;
    public static double ROTATE_SPEED_SLOW   = 0.2;
    public static double ROTATE_SPEED_DEFAULT= 0.4;
    public static double ROTATE_SPEED_FAST   = 0.6;
    public static double TIMEOUT_ROTATE      = 3.0; // seconds

    // member variables
    private Mecabot robot;
    private double robotHeading;
    private String movementStatus;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {

        robot = new Mecabot(hardwareMap);
        robot.initIMU();

        // Add permanent message lines to the Driver Station telemetry
        composeTelemetry();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Init Complete
        telemetry.addData("Gyro Rotate", "All Init Complete, Ready to Start");
        telemetry.addData("Gyro Rotate", "Press X to stop");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Begin calibration, by pivoting the robot in place, to maintain TARGET_HEADING
        // if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        while(!(gamepad1.x) && opModeIsActive()) {
            gyroRotateToHeading(TARGET_HEADING);
        }

        //Stop the robot
        robot.stopDriving();

        // Do not exit, display stats, until the op-mode is stopped by user
        while(opModeIsActive()){
            telemetry.addData("Gyro Rotate PID ", "Complete");
            //Update values defined in composeTelemetry
            telemetry.update();
        }
    }

    /**
     * Rotate the robot to desired angle position using the built in gyro in IMU sensor onboard the REV expansion hub
     *
     * @param targetAngle    The desired target angle position in radians.
     */
    public void gyroRotateToHeading(double targetAngle) {
        gyroRotateToHeading(targetAngle, ROTATE_SPEED_FAST, TIMEOUT_ROTATE);
    }

    /**
     * Rotate the robot to desired angle position using the built in gyro in IMU sensor onboard the REV expansion hub
     * The angle is specified relative to the start position of the robot when the IMU is initialized
     * and gyro angle is initialized to zero radians.
     *
     * @param target   The desired target angle position in radians. Positive value is counter clockwise from initial zero position
     *                      Negative value is clock wise from initial zero. Angle value wraps around at 180 and -180 degrees.
     * @param turnSpeed     The speed at which to drive the motors for the rotation. 0.0 < turnSpeed <= 1.0
     * @param timeout       Max time allowed for the operation to complete
     */
    public void gyroRotateToHeading(double target, double turnSpeed, double timeout) {

        double heading = robot.getZAngle(); // current heading of the robot from gyro (must be initialized in AngleUnit Radians
        double delta = AngleUnit.normalizeRadians(target - heading);
        double direction = Math.signum(delta); // positive angle requires CCW rotation, negative angle requires CW
        double speed = direction * Math.abs(turnSpeed);
        double targetDeg, headingDeg;

        movementStatus = String.format(Locale.US,"Rot Tgt=%.2f | Spd=%1.2f | TO=%1.2f", target, turnSpeed, timeout);
        ElapsedTime runtime = new ElapsedTime();
        // while the robot heading has not reached the target (delta sign flips), and consider reached within a margin
        // Margin is used because the overshoot for even small gyro rotation is 0.15 degrees and higher for larger rotations
        while (opModeIsActive() && ((direction * delta) > 0.0052) && (runtime.seconds() < timeout)) {  // 0.0052 radians = 0.3 degrees

            // slow down linearly for the last N degrees rotation remaining, ROTATE_SPEED_SLOW is required to overcome inertia
            if ((direction * delta) < 0.175) {  // 0.175 Radians = 10 degrees
                speed = direction * ROTATE_SPEED_SLOW;
            }
            // the sign of delta determines the direction of rotation of robot
            robot.setDrivePower(-speed, +speed);
            idle(); // allow some time for the motors to actuate
            heading = robot.getZAngle();
            delta = AngleUnit.normalizeRadians(target - heading);

            // print statistics
            targetDeg = Math.toDegrees(target);
            headingDeg = Math.toDegrees(heading);
            movementStatus = String.format(Locale.US,"Rot Tgt=%.2f | Act=%.2f | Spd=%1.2f | t=%1.2f",
                    targetDeg, headingDeg, turnSpeed, runtime.seconds());
            telemetry.addData("Target", targetDeg);
            telemetry.addData("Heading", headingDeg);
            telemetry.addData("Error", targetDeg - headingDeg);
            telemetry.update();
        }
        robot.stopDriving();
    }

    /*****************************
     * Telemetry setup
     ****************************/
    protected void composeTelemetry() {
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Add here any expensive work that should be done only once just before telemetry update push
                robotHeading = robot.getZAngle();
            }
        });
        telemetry.addLine("IMU ")
                .addData("Rad", "%1.4f", new Func<Double>() {
                    @Override
                    public Double value() { return Math.toDegrees(robotHeading); }
                })
                .addData("Deg", "%3.2f°", new Func<Double>() {
                @Override
                public Double value() { return robotHeading; }
                });
        telemetry.addLine("M ")
                .addData("", new Func<String>() {
                    @Override
                    public String value() {
                        return movementStatus;
                    }
                });

    }
}
