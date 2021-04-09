package org.firstinspires.ftc.teamcode.odometry;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.List;

@Config
public class RealsenseT265CameraLocalizer implements Localizer {

    public static double SLAMRA_TO_ROBOT_OFFSET_X = -0.23; // meters
    public static double SLAMRA_TO_ROBOT_OFFSET_Y = 0; // meters
    public static final double INCHES_TO_METERS = 0.0254;

    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;
    private Transform2d robotOffset = new Transform2d(new Translation2d(SLAMRA_TO_ROBOT_OFFSET_X,SLAMRA_TO_ROBOT_OFFSET_Y), new Rotation2d());
    private T265Camera.CameraUpdate lastReceivedUpdate = null;
    private Pose2d poseEstimate = new Pose2d();
    private Pose2d poseSetByUser = null;
    private boolean updateReceived = false;
    private final Object lock = new Object();

    public RealsenseT265CameraLocalizer(HardwareMap hardwareMap) {
        slamra = new T265Camera(robotOffset, 0, hardwareMap.appContext);
        start();
    }

    public void start() {
        slamra.start((update) -> {
            lastReceivedUpdate = update;
            synchronized (lock) {
                poseEstimate = fromSlamra(update.pose);
                if (!updateReceived) {  // execute upon first update received
                    setPoseToSlamra();
                }
                updateReceived = true;
            }
        });
    }
    public void stop() {
        slamra.stop();
    }
    public boolean getUpdateReceived() {
        return updateReceived;
    }
    // do not lock this method, it will cause deadlock, this is private method only called by synchronized(lock) blocks
    private void setPoseToSlamra() {
        if (poseSetByUser != null) {
            slamra.setPose(toSlamra(poseSetByUser));
            poseEstimate = poseSetByUser;
        }
        poseSetByUser = null;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d newPose) {
        poseSetByUser = newPose;
        synchronized (lock) {
            if (updateReceived) {
                setPoseToSlamra();
            }
        }
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        synchronized (lock) {
            return poseEstimate;
        }
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        ChassisSpeeds velo = lastReceivedUpdate.velocity;
        return new Pose2d(velo.vxMetersPerSecond, velo.vyMetersPerSecond, velo.omegaRadiansPerSecond);
    }

    public T265Camera.PoseConfidence getPoseConfidence() {

        return (lastReceivedUpdate != null) ? lastReceivedUpdate.confidence : T265Camera.PoseConfidence.Failed;
    }

    @Override
    public void update() {
        // nothing to do since the pose updates are received in auto callback
        // and this is not a main class, so status updates for human users will be printed by another class
    }

    /**
     * Returns the positions of the tracking wheels in encoder counts! (not distance units)
     * IMPORTANT: The order of elements is assumed to be 0: LEFT, 1:RIGHT, 2:FRONT odometry wheel
     */
    @NotNull
    protected List<Integer> getWheelPositions() {
        return new ArrayList<>();
    }

    public com.arcrobotics.ftclib.geometry.Pose2d toSlamra(Pose2d rrPose) {
        return new com.arcrobotics.ftclib.geometry.Pose2d(
                new Translation2d(
                rrPose.getX() * INCHES_TO_METERS,
                rrPose.getY() * INCHES_TO_METERS),
                new Rotation2d(rrPose.getHeading())
        );
    }
    public Pose2d fromSlamra(com.arcrobotics.ftclib.geometry.Pose2d ftclibPose) {
        return new Pose2d(
                ftclibPose.getX() / INCHES_TO_METERS,
                ftclibPose.getY() / INCHES_TO_METERS,
                ftclibPose.getHeading()
        );
    }
}
