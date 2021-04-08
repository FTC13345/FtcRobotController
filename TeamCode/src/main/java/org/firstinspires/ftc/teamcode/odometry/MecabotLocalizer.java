package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.odometry.ThreeWheelGyroLocalizer;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Arrays;
import java.util.List;

public class MecabotLocalizer extends ThreeWheelGyroLocalizer {

    public static double TICKS_PER_REV = 8192;   // FTC Team 13345 Mecabot odometry encoder (Rev magnetic encoder) has 8192 ticks per rotation
    public static double WHEEL_RADIUS = 19/25.4; // in | odometry wheel has 38mm diameter = 19mm radius = 0.748 in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    public static double ENCODER_TICKS_PER_INCH = (TICKS_PER_REV) / (2 * Math.PI * WHEEL_RADIUS * GEAR_RATIO);  // 1742.97 FTC Team 13345 Mecabot odometry encoder
    public static double WHEELBASE_SEPARATION_COUNT = 24642; // distance between the left and right wheels
    public static double HORIZONTAL_COUNT_PER_RADIAN = 12919;

    private Encoder leftEncoder, rightEncoder, frontEncoder;
    private BNO055IMU imu;

    public MecabotLocalizer(Encoder left, Encoder right, Encoder front, BNO055IMU bno055IMU) {
        super(  ENCODER_TICKS_PER_INCH,
                WHEELBASE_SEPARATION_COUNT,
                HORIZONTAL_COUNT_PER_RADIAN
        );

        leftEncoder = left;
        rightEncoder = right;
        frontEncoder = front;
        imu = bno055IMU;

        // DONE 13345 | reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        // IMPORTANT: The odometry encoders may be sharing motor ports used for other purpose which sets motor direction
        // Here we override the Encoder direction (software setting) ONLY if needed, without changing motor direction
        leftEncoder.setDirection(Encoder.Direction.FORWARD);  // IMPORTANT: robot forward movement should produce positive encoder count
        rightEncoder.setDirection(Encoder.Direction.FORWARD); // IMPORTANT: robot forward movement should produce positive encoder count
        frontEncoder.setDirection(Encoder.Direction.FORWARD); // IMPORTANT: robot right sideways movement should produce positive encoder count
    }


    /**
     * Returns the positions of the tracking wheels in encoder counts! (not distance units)
     * IMPORTANT: The order of elements is assumed to be 0: LEFT, 1:RIGHT, 2:FRONT odometry wheel
     */
    @NotNull
    @Override
    protected List<Integer> getWheelPositions() {
        return Arrays.asList(
                leftEncoder.getCurrentPosition(),
                rightEncoder.getCurrentPosition(),
                frontEncoder.getCurrentPosition()
        );
    }

    /**
     * Returns the robot's heading in radians as measured by an external sensor (e.g., IMU, gyroscope).
     */
    @Override
    protected double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}
