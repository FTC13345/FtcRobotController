package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.odometry.MecabotLocalizer;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;
import org.firstinspires.ftc.teamcode.odometry.RRTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.odometry.RealsenseT265CameraLocalizer;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.Encoder;

public abstract class FtcRobotDAL extends Mecabot {

    // mecanum drive train
    protected RRMecanumDrive rrmdrive;
    protected MecabotDrive mcdrive;         // soon to be disabled since we are using RoadRunner exclusively
    protected LinearOpMode myOpMode;       // Access to the OpMode object
    protected FtcDashboard dashboard;
    protected Telemetry telemetry;
    // a bunch of localizers since we want to experiment which ones will work reliably
    protected RealsenseT265CameraLocalizer cameraLocalizer;
    protected RRTrackingWheelLocalizer roadrunnerLocalizer;
    protected MecabotLocalizer triWheelGyroLocalizer;
    protected MecabotLocalizer triWheelLocalizer;
    protected OdometryGlobalPosition globalPosition;

    protected abstract void gameUpdate(TelemetryPacket packet);

    // constructor
    public FtcRobotDAL(HardwareMap hardwareMap, LinearOpMode opMode) {
        super(hardwareMap);
        myOpMode = opMode;
        telemetry = opMode.telemetry;
        dashboard = FtcDashboard.getInstance();
        // odometry Localizers must be created before Mecanum Drive objects
        initOdometry(hardwareMap);
        rrmdrive = new RRMecanumDrive(hardwareMap, roadrunnerLocalizer, opMode);
        mcdrive = new MecabotDrive(this, globalPosition, opMode);  // soon to be disabled since we are using RoadRunner exclusively
    }

    // mecanum drivetrain used by the Robot
    public RRMecanumDrive getRRMdrive() { return rrmdrive; }

    /*
     * Odometry and Pose related methods.
     */
    private void initOdometry(HardwareMap hardwareMap) {

        cameraLocalizer = new RealsenseT265CameraLocalizer(hardwareMap);

        // IMPORTANT: The odometry encoders may be sharing motor ports used for other purpose which sets motor direction
        // The Encoder direction (software setting) should be manipulated by localizer AS NEEDED, without changing motor direction
        Encoder leftODwheel = new Encoder(hardwareMap.get(DcMotorEx.class, "leftODwheel"));
        Encoder rightODwheel = new Encoder(hardwareMap.get(DcMotorEx.class, "rightODwheel"));
        Encoder frontODWheel = new Encoder(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
        // reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        // IMPORTANT: The odometry encoders may be sharing motor ports used for other purpose which sets motor direction
        // Here we override the Encoder direction (software setting) ONLY if needed, without changing motor direction
        leftODwheel.setDirection(Encoder.Direction.FORWARD);  // IMPORTANT: robot forward movement should produce positive encoder count
        rightODwheel.setDirection(Encoder.Direction.FORWARD); // IMPORTANT: robot forward movement should produce positive encoder count
        frontODWheel.setDirection(Encoder.Direction.FORWARD); // IMPORTANT: robot right sideways movement should produce positive encoder count

        roadrunnerLocalizer = new RRTrackingWheelLocalizer(leftODwheel, rightODwheel, frontODWheel);

        triWheelGyroLocalizer = new MecabotLocalizer(leftODwheel, rightODwheel, frontODWheel, imu);

        //Create and start GlobalPosition thread to constantly update the global position coordinates.
        globalPosition = new OdometryGlobalPosition(leftODwheel, rightODwheel, frontODWheel);
        myOpMode.telemetry.addData("Wheelbase Separation", globalPosition.getWheelbaseSeparationCount());
        myOpMode.telemetry.addData("Horizontal Count Per Radian", globalPosition.getHorizonalCountPerRadian());

    }

    public void resetOdometryEncoder() {
        globalPosition.resetOdometryEncoder();
    }

    // We need to set Pose in multiple localizers that we are running for comparison
    public void setPose(Pose2d pose) {
        // rrmdrive.setPoseEstimate(pose);  // temp disabled since RRMDrive may be using any one of below multiple localizers
        cameraLocalizer.setPoseEstimate(pose);
        roadrunnerLocalizer.setPoseEstimate(pose);
        triWheelGyroLocalizer.setPoseEstimate(pose);
        globalPosition.setGlobalPosition(pose.getX(), pose.getY(), pose.getHeading());
    }

    public void start() {
        // start any localizer and driving threads here
        // RealsenseT265CameraLocalizer starts itself upon creation
        globalPosition.start();
    }
    public void stop() {
        // stop any localizer and driving threads here
        cameraLocalizer.stop();
        globalPosition.stop();
    }
    public void update() {
        // Update Pose Estimate in the localizers
        // cameraLocalizer does not need update call, it has callback to return Pose
        // globalPosition does not need update call, it runs its own thread
        roadrunnerLocalizer.update();
        triWheelGyroLocalizer.update();

        // FTC Dashboard map display and telemetry
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        // draw robot position from each localizer
        fieldOverlay.setStroke("#D2691E");  // CHOCOATE
        DashboardUtil.drawRobot(fieldOverlay, cameraLocalizer.getPoseEstimate());
        fieldOverlay.setStroke("#4CAF50");  // GREEN
        DashboardUtil.drawRobot(fieldOverlay, roadrunnerLocalizer.getPoseEstimate());
        fieldOverlay.setStroke("#3F51B5");  // BLUE
        DashboardUtil.drawRobot(fieldOverlay, triWheelGyroLocalizer.getPoseEstimate());
        fieldOverlay.setStroke("#B800B8");  // MAGENTA
        DashboardUtil.drawRobot(fieldOverlay, new Pose2d(globalPosition.getX(), globalPosition.getY(), globalPosition.getHeading()));

        // Driver Station Telemetry
        telemetry.addData("Slamra Confidence", RealsenseT265CameraLocalizer.PoseConfidenceLabel[cameraLocalizer.getPoseConfidence().ordinal()]);

        // Call the sub-classes to collect any dashboard or telemetry updates (also any other periodic processing)
        gameUpdate(packet);
        // send all collected data to dashboard
        dashboard.sendTelemetryPacket(packet);
        // telemetry update to push to driver station will be called by the main Op Mode
    }

    /*
     * Utility functions useful for subsystem motors other than the drive train
     */
    public void waitUntilMotorBusy(DcMotor motor) {
        ElapsedTime runTime = new ElapsedTime();
        while (myOpMode.opModeIsActive() && motor.isBusy() && (runTime.seconds() < MecabotDrive.TIMEOUT_DEFAULT)){
            myOpMode.idle();
        }
    }
    public void motorRunToPosition(DcMotor motor, int position, double speed) {
        if (!myOpMode.opModeIsActive()) {
            return;
        }
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);
        // disabled, do not wait for motor to reach target position, this is necessary to not block the thread
        //waitUntilMotorBusy(motor);
        // caller must realize that the call is asynchronous and motor will take timee after return from this call
    }

    /*****************************
     * Telemetry setup
     ****************************/
    protected void composeTelemetry() {
        telemetry.addAction(() -> {
            // Add here any expensive work that should be done only once just before telemetry update push
        });
        telemetry.addLine("CAM ")
                .addData("X", "%.1f", () -> cameraLocalizer.getPoseEstimate().getX())
                .addData("Y", "%.1f", () -> cameraLocalizer.getPoseEstimate().getY())
                .addData("Head", "%.2f°", () -> Math.toDegrees(cameraLocalizer.getPoseEstimate().getHeading()));
        telemetry.addLine("GYR ")
                .addData("X", "%.1f", () -> triWheelGyroLocalizer.getPoseEstimate().getX())
                .addData("Y", "%.1f", () -> triWheelGyroLocalizer.getPoseEstimate().getY())
                .addData("Head", "%.2f°", () -> Math.toDegrees(triWheelGyroLocalizer.getPoseEstimate().getHeading()))
                .addData("IMU", "%.2f°", () -> Math.toDegrees(triWheelGyroLocalizer.getExternalHeading()));
        telemetry.addLine("RRN ")
                .addData("X", "%.1f", () -> roadrunnerLocalizer.getPoseEstimate().getX())
                .addData("Y", "%.1f", () -> roadrunnerLocalizer.getPoseEstimate().getY())
                .addData("Head", "%.2f°", () -> Math.toDegrees(roadrunnerLocalizer.getPoseEstimate().getHeading()));
        telemetry.addLine("ODO ")
                .addData("X", "%.1f", () -> globalPosition.getX())
                .addData("Y", "%.1f", () -> globalPosition.getY())
                .addData("Head", "%.2f°", () -> Math.toDegrees(globalPosition.getHeading()));
        telemetry.addLine("OD Count ")
                .addData("L", "%6.0f", () -> globalPosition.getVerticalLeftCount())
                .addData("R", "%6.0f", () -> globalPosition.getVerticalRightCount())
                .addData("X", "%6.0f", () -> globalPosition.getHorizontalCount());
        telemetry.addLine("Drive ")
                .addData("LF", "%5d", () -> leftFrontDrive.getCurrentPosition())
                .addData("LB", "%5d", () -> leftBackDrive.getCurrentPosition())
                .addData("RF", "%5d", () -> rightFrontDrive.getCurrentPosition())
                .addData("RB", "%5d", () -> rightBackDrive.getCurrentPosition());
    }
}
