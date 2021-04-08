package org.firstinspires.ftc.teamcode.odometry;

import android.app.slice.Slice;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 * Modified by Vishesh Goyal on 12/24/2019 for the Skystone Season
 * Adaptation to Team13345 Mecabot hardware using goBilda strafer kit v2 on 10/25/2020 for the Ultimate Goal season
 * - Detailed comments to explain the calculation of wheel base separation. Improved variable names for better understanding.
 * - Bug fixes in code and comments
 *
 * Odometry system calibration. Run this OpMode to generate the necessary constants to calculate the robot's global position on the field.
 * The Global Positioning Algorithm will not function and will throw an error if this program is not run first
 */
@TeleOp(name = "Odometry System Calibration", group = "Calibration")
@Disabled
public class OdometryCalibration extends LinearOpMode {

    final double PIVOT_SPEED = 0.4;
    final double SLOW_SPEED = 0.2;
    final double rotations = 10;

    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    //IMU Sensor
    BNO055IMU imu;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE FOR EACH ROBOT AND NEED TO BE UPDATED HERE
    String rfName = "rightFrontDrive", rbName = "rightBackDrive", lfName = "leftFrontDrive", lbName = "leftBackDrive";
    String verticalLeftEncoderName = "leftODwheel", verticalRightEncoderName = "rightODwheel", horizontalEncoderName = "intakeMotor";

    // THIS WILL CHANGE FOR EACH ROBOT AND NEED TO BE UPDATED HERE (change the ticks per rotation and the wheel diameter
    // The amount of encoder ticks for each inch the robot moves.
    final double ENCODER_COUNT_PER_INCH = (8192.0f * 25.4f) / (Math.PI * 38.0f);  // 1742.97 FTC Team 13345 Mecabot odometry encoder (Rev magnetic encoder) has 8192 ticks per rotation, odometry wheel has 38mm diameter

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalCountPerRadianFile = AppUtil.getInstance().getSettingsFile("horizontalCountPerRadian.txt");

    @Override
    public void runOpMode() {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        //Initialize IMU hardware map value. PLEASE UPDATE THIS VALUE TO MATCH YOUR CONFIGURATION
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        telemetry.addData("Odometry Calibration", "IMU Init Complete");
        telemetry.clear();

        //Odometry System Calibration Init Complete
        telemetry.addData("Odometry Calibration", "All Init Complete, Ready to Start");
        telemetry.addData("Odometry Calibration", "Press X to stop");
        telemetry.update();

        waitForStart();

        boolean done = false;
        double target = rotations * 360; // degrees
        double speed = PIVOT_SPEED;
        double headingCurrent;
        double headingLast = 0;
        double headingDelta;
        double headingCumulative = 0;

        // Begin calibration, by pivoting the robot in place, in this setup rotate right is positive angle
        // if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        do {

            headingCurrent = getZAngle();
            headingDelta = MathFunctions.angleWrap(headingCurrent - headingLast);
            headingCumulative += headingDelta;
            headingLast = headingCurrent;

            if (Math.abs(headingCumulative) > target - 20) {
                speed = SLOW_SPEED;
            }
            setPowerAll(speed); // rotate

            telemetry.addData("IMU Angle", getZAngle());
            telemetry.addData("Total Rotation", headingCumulative);
            telemetry.update();

        } while(!(gamepad1.x) && (Math.abs(headingCumulative) < target) && opModeIsActive());

        //Stop the robot
        setPowerAll(0.0);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.milliseconds() < 2000 && opModeIsActive()){
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.addData("Total Rotation", headingCumulative);
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = getZAngle();

        // THIS WILL CHANGE FOR EACH ROBOT AND NEED TO BE UPDATED HERE (only sign change)
        // Since the orientation of encoders on each side is opposite, one of the encoder value
        // needs to be reversed so that both side encoders produced positive ticks with forward movement.
        // Horizontal encoder ticks may also need sign reversal, in this code clockwise rotation of robot should produce positive tick count
        double verticalLeftCount = verticalLeft.getCurrentPosition();
        double verticalRightCount = verticalRight.getCurrentPosition();
        double horizontalCount = horizontal.getCurrentPosition();

        // The Robot pivoted around its own center for a certain angle and we recorded the encoder ticks on left and right
        // wheel base separation = sum of radius of left arc and radius of right arc around the pivot point
        // Note that: length of arc for 1 radian angle = radius of circle
        // Therefore wheel base separation = sum of left arc length and right arc length for 1 radian rotation
        double verticalEncoderTicks = Math.abs(verticalLeftCount) + (Math.abs(verticalRightCount));
        double verticalEncoderTicksPerDegree = verticalEncoderTicks/headingCumulative;
        double verticalEncoderTicksPerRadian = (180*verticalEncoderTicks)/(Math.PI*headingCumulative);
        double wheelBaseSeparationInches = verticalEncoderTicksPerRadian/ ENCODER_COUNT_PER_INCH;

        double horizontalCountPerRadian = Math.abs(horizontalCount)/Math.toRadians(headingCumulative);

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparationInches));
        ReadWriteFile.writeFile(horizontalCountPerRadianFile, String.valueOf(horizontalCountPerRadian));

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration ", "Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation (inches)", "%.2f", wheelBaseSeparationInches);
            telemetry.addData("Horizontal Encoder Ticks per Radian", "%.0f", horizontalCountPerRadian);
            telemetry.addData("Calculations", "");

            //Display raw values
            telemetry.addData("Total Rotation (IMU degrees)", "%.2f°", headingCumulative);
            telemetry.addData("Vertical Left Encoder Ticks", "%.0f", verticalLeftCount);
            telemetry.addData("Vertical Right Encoder Ticks", "%.0f", verticalRightCount);
            telemetry.addData("Vertical Encoder Ticks per Degree", "%.2f", verticalEncoderTicksPerDegree);
            telemetry.addData("Vertical Encoder Ticks per Radian", "%.2f", verticalEncoderTicksPerRadian);
            telemetry.addData("Horizontal Encoder Ticks", "%.0f", horizontalCount);

            //Update values
            telemetry.update();
        }
    }

    private void initHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // THIS WILL CHANGE FOR EACH ROBOT AND NEED TO BE UPDATED HERE
        // Left side set to REVERSE if using AndyMark or goBilda 5202 yellow jacket motors
        left_front.setDirection(DcMotor.Direction.REVERSE);
        left_back.setDirection(DcMotor.Direction.REVERSE);
        // Right side set to FORWARD if using AndyMark or goBilda 5202 yellow jacket motors
        right_front.setDirection(DcMotor.Direction.FORWARD);
        right_back.setDirection(DcMotor.Direction.FORWARD);
        // OD encoders are using any available ports, where motor is not driven by encoder.
        // Set direction to produce positive count increment when robot moves forward or right
        verticalLeft.setDirection(DcMotor.Direction.FORWARD);
        verticalRight.setDirection(DcMotor.Direction.FORWARD);
        horizontal.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Odometry Calibration", "Hardware Map Init Complete");
        telemetry.update();

    }

    /**
     * Gets the orientation of the robot using the REV IMU
     * This code assumes that clockwise rotation is positive angle, thus reverse sign of value returned by IMU
     * @return the angle of the robot
     */
    private double getZAngle(){

        return (-imu.getAngularOrientation().firstAngle);
    }

    /**
     * Sets power to all four drive motors
     * @param speed power for all the  motors
     */
    private void setPowerAll(double speed){
        right_front.setPower(-speed);
        right_back.setPower(-speed);
        left_front.setPower(+speed);
        left_back.setPower(+speed);
    }

}
