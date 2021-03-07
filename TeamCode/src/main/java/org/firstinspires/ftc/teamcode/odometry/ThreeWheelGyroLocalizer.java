package org.firstinspires.ftc.teamcode.odometry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.List;

public abstract class ThreeWheelGyroLocalizer implements Localizer {

    protected boolean useExternalHeading = true;

    private Pose2d poseEstimate;
    private double robotXcount = 0, robotYcount = 0, robotHeading = 0;
    private double headingOffset = 0.0;
    protected List<Integer> lastWheelPositions = new ArrayList<>();
    private final double ENCODER_TICKS_PER_INCH;
    private final double WHEELBASE_SEPARATION_COUNT;
    private final double HORIZONTAL_COUNT_PER_RADIAN;

    public ThreeWheelGyroLocalizer(double encoderTicksPerInch, double wheelbaseSeparationTicks, double frontEncoderTicksPerRadian) {
        ENCODER_TICKS_PER_INCH = encoderTicksPerInch;
        WHEELBASE_SEPARATION_COUNT = wheelbaseSeparationTicks;
        HORIZONTAL_COUNT_PER_RADIAN = frontEncoderTicksPerRadian;
    }

    protected double getExternalHeading() {
        return Angle.normDelta(getRawExternalHeading() + headingOffset);
    }
    protected void setExternalHeading(double value) {
        headingOffset = Angle.normDelta(value - getRawExternalHeading());
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose) {
        poseEstimate = pose;
        robotXcount = pose.getX() * ENCODER_TICKS_PER_INCH;
        robotYcount = pose.getY() * ENCODER_TICKS_PER_INCH;
        robotHeading = pose.getHeading();
        if (useExternalHeading) {
            setExternalHeading(robotHeading);
        }
        lastWheelPositions = new ArrayList<>();
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {

        //Get Current Positions
        List<Integer> wheelPositions = getWheelPositions();
        if (lastWheelPositions.isEmpty()) {
            return;
        }

        // calculate the positional displacement in vertical direction (forward movement of robot)
        int leftWheelDelta = wheelPositions.get(0) - lastWheelPositions.get(0);
        int rightWheelDelta = wheelPositions.get(1) - lastWheelPositions.get(1);
        // calculate the positional displacement only in horizontal direction (sideways movement of robot)
        int frontWheelDelta = wheelPositions.get(2) - lastWheelPositions.get(2);

        // Calculate Robot heading delta
        // These formulas assume that robotHeading is positive when Robot is turning counter-clockwise
        // All local variables are a signed value, representing change or angle direction
        double deltaHeading;
        if (useExternalHeading) {
            double extHeading = getExternalHeading();
            deltaHeading = Angle.normDelta(extHeading - robotHeading);
            robotHeading = extHeading;
        } else {
            deltaHeading = (rightWheelDelta - leftWheelDelta) / (WHEELBASE_SEPARATION_COUNT);
            robotHeading = Angle.normDelta(robotHeading + deltaHeading);
        }

        // Get the components of the motion
        // p is the vector of forward movement of the Robot, direction parallel to drivetrain wheels
        // n is the vector of normal (sideways) movement of the Robot, direction perpendicular to drivetrain wheels
        double p = ((double)(rightWheelDelta + leftWheelDelta) / 2);

        // The horizontal encoder (or cross encoder) is mounted on front side of robot. A left turn (positive angle rotation) includes
        // (1) horizontal encoder travel left (negative increment) and
        // (2) swing movement to left around the center pivot of robot (positive increment).
        // The 2 terms for horizontal change must be added for net positional displacement
        // If horizontal encoder was mounted on the back of the robot then the 2nd term needed to be subtracted from 1st term.
        double n = frontWheelDelta + (deltaHeading * HORIZONTAL_COUNT_PER_RADIAN);

        // Calculate and update the position using Vector projection formula
        // These formulas assume robotHeading is measured from X-Axis turning counter-clockwise, same as angle theta in a polar coordinate system
        robotXcount += (p*Math.cos(robotHeading) + n*Math.sin(robotHeading));
        robotYcount += (p*Math.sin(robotHeading) - n*Math.cos(robotHeading));

        poseEstimate = new Pose2d(robotXcount/ENCODER_TICKS_PER_INCH, robotYcount/ENCODER_TICKS_PER_INCH, robotHeading);

        lastWheelPositions = wheelPositions;
    }

    /**
     * Returns the positions of the tracking wheels in encoder counts! (not distance units)
     */
    @NotNull
    protected abstract List<Integer> getWheelPositions();

    /**
     * Returns the robot's heading in radians as measured by an external sensor (e.g., IMU, gyroscope).
     */
    protected abstract double getRawExternalHeading();

}
