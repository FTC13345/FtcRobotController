package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;
import com.spartronics4915.lib.T265Camera.CameraUpdate;
import java.util.function.Consumer;
import java.util.Locale;
/*
 *
 *
 *
 */

@TeleOp(name="T265 Camera 13345", group="Test")
//@Disabled                            // Comment this out to add to the opmode list
public class T265Camera13345 extends LinearOpMode
{
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private Transform2d robotOffset = new Transform2d(new Translation2d(-0.23,0), new Rotation2d(0));

    private ElapsedTime timer = new ElapsedTime();
    private String[] label = {"FAILED", "LOW", "MEDIUM", "HIGH"};
    private int robotRadius = 9; // inches
    private double detectionTime = 0;
    private int count = 0;

    @Override
    public void runOpMode() {

        // Wait for the start button to be pressed.
        waitForStart();

        timer.reset();
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.0, hardwareMap.appContext);
            count++;
        }
        slamra.setPose(new Pose2d(-62 * 0.0254, +32 * 0.0254, new Rotation2d(0)));
        slamra.start();

        // Loop until we are asked to stop
        while (opModeIsActive()) {
            T265Camera.CameraUpdate update = slamra.getLastReceivedCameraUpdate();
            if (update == null) return;

            sendTelemetry(update);
        }

        // cleanup before exit
        slamra.stop();
    }

    private void sendTelemetry(CameraUpdate update) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        // We divide by 0.0254 to convert meters to inches
        Pose2d pose = update.pose;
        Translation2d translation = new Translation2d(pose.getTranslation().getX() / 0.0254, pose.getTranslation().getY() / 0.0254);
        Rotation2d rotation = pose.getRotation();

        String telemetryLine = String.format(Locale.US, "(%02.2f, %02.2f) H: %03.2f (deg)", translation.getX(), translation.getY(), rotation.getDegrees());
        packet.put("Pose", telemetryLine);
        packet.put("Velocity", update.velocity);
        packet.put("Confidence", label[update.confidence.ordinal()]);
        if ((detectionTime == 0) && (update.confidence != T265Camera.PoseConfidence.Failed)) {
            detectionTime = timer.seconds();
        }
        telemetryLine = String.format(Locale.US, "%.3f | %.3f", timer.seconds(), detectionTime);
        packet.put("Elapsed Time", telemetryLine);
        packet.put("Camera constructed", count);

        field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
        field.strokeLine(x1, y1, x2, y2);

        dashboard.sendTelemetryPacket(packet);
    }

}
