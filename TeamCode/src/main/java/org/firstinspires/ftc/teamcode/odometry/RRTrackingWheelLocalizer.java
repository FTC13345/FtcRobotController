package org.firstinspires.ftc.teamcode.odometry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class RRTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;   // FTC Team 13345 Mecabot odometry encoder (Rev magnetic encoder) has 8192 ticks per rotation
    public static double WHEEL_RADIUS = 19/25.4; // in | odometry wheel has 38mm diameter = 19mm radius = 0.748 in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 14.2; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 7.4; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 1.008;    // measured using LocalizationTest tuning opmode
    public static double Y_MULTIPLIER = 1.0;       // measured using LocalizationTest tuning opmode

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public RRTrackingWheelLocalizer(Encoder left, Encoder right, Encoder front) {
        super(Arrays.asList(
                new Pose2d(-0.63, 7.28, 0), // left
                new Pose2d(-0.63, -6.92, 0), // right
                new Pose2d(FORWARD_OFFSET, -0.47, Math.toRadians(-90)) // front
        ));

        leftEncoder = left;
        rightEncoder = right;
        frontEncoder = front;

        // PLEASE UPDATE THESE VALUES TO MATCH YOUR ODOMETRY HARDWARE *AND* the DCMOTOR DIRECTION (FORWARD/REVERSE) CONFIGURATION
        // IMPORTANT: The odometry encoders may be sharing motor ports used for other purpose which sets motor direction
        // Here we override the Encoder direction (software setting) ONLY if needed, without changing motor direction
        //leftEncoder.setDirection(Encoder.Direction.REVERSE);  // IMPORTANT: robot forward movement should produce positive encoder count
        //rightEncoder.setDirection(Encoder.Direction.REVERSE); // IMPORTANT: robot forward movement should produce positive encoder count
        //frontEncoder.setDirection(Encoder.Direction.REVERSE); // IMPORTANT: robot right sideways movement should produce positive encoder count
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // 13345 Done | If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()),
                encoderTicksToInches(frontEncoder.getCorrectedVelocity())
        );
    }
}
