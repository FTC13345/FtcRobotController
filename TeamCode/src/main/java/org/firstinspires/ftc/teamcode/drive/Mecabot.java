/* Copyright (c) 2019-20 FIRST FTC Team 13345 POLARIS. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Mecabot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note: The names can be found in the init() method where literal strings are used to initialize hardware.
 *
 */
public class Mecabot {
    //constants here
    public static final double LENGTH = 17.0;   // exact length 17.2 in see https://www.gobilda.com/strafer-chassis-kit-3209-0001-0002/
    public static final double WIDTH = 17.0;    // exact length 17.4 in see https://www.gobilda.com/strafer-chassis-kit-3209-0001-0002/
    public static final double HALF_WIDTH = WIDTH / 2;

    /* local OpMode members. */
    // The hardware map obtained from OpMode
    HardwareMap hwMap;
    // The IMU sensor object
    protected BNO055IMU imu;
    // drive train motors
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;

    // Lights control
    public RevBlinkinLedDriver lights;
    public RevBlinkinLedDriver.BlinkinPattern pattern;

    // Robot front and rear can be flipped for driving purposes
    // Define enum constant for whether robot is in NORMAL forward mode or in REVERSE mode
    enum DIRECTION {FORWARD, REVERSE}
    DIRECTION direction;

    /*
     * Robot front facing direction toggle methods. Robot FRONT direction can be flipped.
     * This is important to understand, to avoid unexpected behavior.
     * When Robot front direction is toggled from FORWARD (INTAKE is front) to REVERSE,
     * the direction change is achieved by changing only 2 underlying methods,
     * all other code is oblivious of this direction reversal.
     * @see MecaBot#setTargetPosition()
     * @see MecaBot#driveWheels()
     *
     * MOST IMPORTANT: All driving and move methods must call one of the above methods,
     * and must NOT set drivetrain motor power directly. Specifically the methods
     * @see MecaBot#SetDrivePower()  do NOT handle frontFace flipping.
     * They are used for non-directional movement, such as gyro rotation.
     */
    public DIRECTION getDirection() {
        return direction;
    }
    public boolean isDirectionForward() {
        return (direction == DIRECTION.FORWARD);
    }
    public boolean isDirectionReverse() {
        return (direction == DIRECTION.REVERSE);
    }
    public void setDirectionForward() {
        direction = DIRECTION.FORWARD;
        setLightGreen();
    }
    public void setDirectionReverse() {
        direction = DIRECTION.REVERSE;
        setLightRed();
    }
    public String getDirectionStr() {
        return ((direction == DIRECTION.FORWARD) ? "FORWARD" : "REVERSE");
    }


    /* Constructor */
    public Mecabot(HardwareMap ahwMap) {
        direction = DIRECTION.FORWARD;
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        // imu.initialize(BNO055IMU.Parameters) must be called otherwise gyro readings will be zero
        // we do not initialize in this class because side effect is to reset gyro heading to zero
        // the op-mode main application code should decided when we want to initialize or not
        // for e.g. after Autonomous op-mode the Tele op-mode may want to continue without reset

        /* Define and Initialize Motors and Servos */

        // Drivetrain motors
        leftFrontDrive = hwMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hwMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hwMap.get(DcMotor.class, "rightBackDrive");
        // Left side set to REVERSE if using AndyMark or goBilda 5202 yellow jacket motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        // Right side set to FORWARD if using AndyMark or goBilda 5202 yellow jacket motors
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        stopDriving();
        // RUN_USING_ENCODERS if encoders are installed.
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // lights
        lights = hwMap.get(RevBlinkinLedDriver.class, "lights");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        lights.setPattern(pattern);

    }

    /*
     * Initialize the IMU for AngleUnit RADIANS and with default parameter values
     * This method is mandatory to be called once after power up, otherwise the angle values returned == 0
     */
    public void initIMU() {
        // Set up the parameters with which we will use our IMU.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    /**
     * Gets the orientation of the robot using the REV IMU
     * @return the angle of the robot
     */
    public double getZAngle() {
        return imu.getAngularOrientation().firstAngle;
    }

    /*
     * Driving movement methods
     */

    /**
     * Set encoder position for each wheel in preparation for a RUN_USING_ENCODERS movement
     * This method DOES take into account the frontFace of the robot, whether Intake or Liftarm
     * and corresponding to robot front direction, sets the target encoder value on the wheels
     * Note: {@link Mecabot#driveWheels(double, double, double, double)} also accounts for frontFace
     * Note: {@link Mecabot#setDrivePower(double)} does NOT consider frontFace of the robot, the most
     * common use of that method is after this method has been called
     * @param leftFront  Left front wheel target encoder count
     * @param leftBack   Left back wheel target encoder count
     * @param rightFront Right front wheel target encoder count
     * @param rightBack  Right back wheel target encoder count
     */
    public void setTargetPosition(int leftFront, int leftBack, int rightFront, int rightBack) {
        if (direction == DIRECTION.FORWARD) {
            leftFrontDrive.setTargetPosition(leftFront);
            leftBackDrive.setTargetPosition(leftBack);
            rightFrontDrive.setTargetPosition(rightFront);
            rightBackDrive.setTargetPosition(rightBack);
        }
        else { // (direction == DIRECTION.REVERSE)
            leftFrontDrive.setTargetPosition(-rightBack);
            leftBackDrive.setTargetPosition(-rightFront);
            rightFrontDrive.setTargetPosition(-leftBack);
            rightBackDrive.setTargetPosition(-leftFront);
        }
    }

    public void setDriveMode(DcMotor.RunMode runMode) {
        leftBackDrive.setMode(runMode);
        leftFrontDrive.setMode(runMode);
        rightBackDrive.setMode(runMode);
        rightFrontDrive.setMode(runMode);

    }
    public void resetDriveEncoder() {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setDrivePower(double speed) {
        speed = Range.clip( speed, -1.0, 1.0);
        leftFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);
    }

    public void setDrivePower(double leftSpeed, double rightSpeed) {
        leftSpeed = Range.clip( leftSpeed, -1.0, 1.0);
        rightSpeed = Range.clip( rightSpeed, -1.0, 1.0);
        leftFrontDrive.setPower(leftSpeed);
        leftBackDrive.setPower(leftSpeed);
        rightFrontDrive.setPower(rightSpeed);
        rightBackDrive.setPower(rightSpeed);
    }

    public void stopDriving() {
        this.setDrivePower(0);
    }

    /**
     * Tank mode drives in forward (or backward) direction only, no mecanum sideways movement
     * The turns are achieved by speed differential between left and right side wheels
     * Sign value of speed parameters controls direction. +ve driveSpeed means forward and
     * +ve turnSpeed means turn left (since turn angle theta is +ve when counter clockwise from X-axis)
     *
     * @param driveSpeed    forward speed specified within range [-1.0, 1.0]
     * @param turnSpeed     turning speed specified within range [-1.0, 1.0]
     */
    public void driveTank(double driveSpeed, double turnSpeed) {

        driveSpeed = Range.clip(driveSpeed, -1.0, 1.0);

        // basic forward/backwards, run motors at drive speed, negative speed is reverse direction
        double leftFront = driveSpeed;
        double leftBack = driveSpeed;
        double rightFront = driveSpeed;
        double rightBack = driveSpeed;

        // left turn is positive turnSpeed, right turn is negative turnSpeed
        // to turn left, add turnSpeed to right motors, subtract from left motors
        // to turn right, same code applies, negative values cause right turn automatically
        turnSpeed = Range.clip(turnSpeed, -1.0, 1.0);
        leftFront -= turnSpeed;
        leftBack -= turnSpeed;
        rightFront += turnSpeed;
        rightBack += turnSpeed;

        driveWheels(leftFront, leftBack, rightFront, rightBack);
    }

    public void driveMecanum(double sideSpeed) {
        // if we want to move right sideways, sideSpeed value is positive
        // right inside
        double rightFront = -sideSpeed;
        double rightBack = +sideSpeed;

        // left outside
        double leftFront = +sideSpeed;
        double leftBack = -sideSpeed;

        // if we want to move left, its same code as above, sideSpeed value is negative

        driveWheels(leftFront, leftBack, rightFront, rightBack);
    }

    /**
     * Drives the robot diagonally
     * Positive values of speed drive it forwards and right
     * Negative values of speed drive it forwards and left
     * @param speed robot power or speed
     */
    public void driveDiagonal(double speed) {
        double leftFront = 0;
        double leftBack = 0;
        double rightFront = 0;
        double rightBack = 0;

        if (speed > 0) {
            leftFront = speed;
            rightBack = speed;
        }
        else if (speed < 0) {
            leftBack = Math.abs(speed);
            rightFront = Math.abs(speed);
        }

        driveWheels(leftFront, leftBack, rightFront, rightBack);
    }


    /**
     * Set drive wheels power to specified values, which are normalized to -1.0 <= power <= 1.0 range
     * This method DOES take into account the frontFace of the robot, whether Intake or Liftarm
     * and corresponding to robot front direction, sets the power on front/back wheels consistent with frontFace
     * Note: {@link Mecabot#setTargetPosition(int, int, int, int)} also accounts for frontFace
     * Note: {@link Mecabot#setDrivePower(double)} does NOT consider frontFace of the robot, the most
     * common use of that method is after {@link Mecabot#setTargetPosition(int, int, int, int)} has been called
     * @param leftFront  Left front wheel target encoder count
     * @param leftBack   Left back wheel target encoder count
     * @param rightFront Right front wheel target encoder count
     * @param rightBack  Right back wheel target encoder count
     */
    public void driveWheels(double leftFront, double leftBack, double rightFront, double rightBack) {
        // find the highest power motor and divide all motors by that to preserve the ratio
        // while also keeping the maximum power at 1
        double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(leftBack)), Math.max(Math.abs(rightFront), Math.abs(rightBack)));
        if (max > 1.0) {
            leftFront /= max;
            leftBack /= max;
            rightFront /= max;
            rightBack /= max;
        }

        //set drive train motor's power to the values calculated
        if (direction == DIRECTION.FORWARD) {
            leftFrontDrive.setPower(leftFront);
            leftBackDrive.setPower(leftBack);
            rightFrontDrive.setPower(rightFront);
            rightBackDrive.setPower(rightBack);
        }
        else { // (direction == DIRECTION.REVERSE)
            leftFrontDrive.setPower(-rightBack);
            leftBackDrive.setPower(-rightFront);
            rightFrontDrive.setPower(-leftBack);
            rightBackDrive.setPower(-leftFront);
        }
    }

    // set light color methods
    public void setBlinkinLedPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        lights.setPattern(pattern);
    }
    public void setLightGreen() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
        lights.setPattern(pattern);
    }
    public void setLightRed() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE;
        lights.setPattern(pattern);
    }
    public void setSlowBlue() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE;
        lights.setPattern(pattern);
    }
    public void setFastBlue() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
        lights.setPattern(pattern);
    }


}

