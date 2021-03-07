package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.MecabotDrive;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TeleOpDriver;

import static org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.*;
import static org.firstinspires.ftc.teamcode.drive.RRMecanumDrive.savedPose;

/* Button Controls (programmers please keep this list up to date)
 * All buttons coded in this file are on GamePad2 unless specified otherwise
 * START + X = Reset Position, odometry, IMU, Lift intake, wobble arm encoder
 *
 * A = Toggle flywheel
 * B = Shoot Ring
 * X = Tilt Shooting Platform for High Goal
 * Y = Tilt Shooting Platform for PowerShot
 *
 * RIGHT_TRIGGER    = Run Intake forward (such in rings)
 * LEFT_TRIGGER     = Run Intake in reverse (eject rings)
 *
 * RIGHT_JOYSTICK_Y = Wobble Arm Manual position control
 * RIGHT_BUMPER     = Wobble Finger Close
 * LEFT_BUMPER      = Wobble Finger Open
 *
 * DPAD_UP          = Wobble Arm UP to Next Position
 * DPAD_DOWN        = Wobble Arm DOWN to Next Position
 * DPAD_RIGHT       = Rotate Shooter towards Goal
 * DPAD_LEFT        = Rotate Intake towards Goal
 *
 * START + RIGHT_BUMPER = Ignore motor stops (wobble arm)
 * START + LEFT_BUMPER = Restore motor stops (wobble arm)
 */

@TeleOp(name = "UGoal TeleOp", group="QT")
public class UGoalTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    RRMecanumDrive rrmdrive;
    MecabotDrive mcdrive;
    UGoalRobot robot;
    OdometryGlobalPosition globalPosition;
    TeleOpDriver driver;

    //Button debounce
    private boolean gamepad2ADebounce = false;
    private boolean gamepad2DpadDebounce = false;

    enum WOBBLE_ARM_MODE { BUTTONS, JOYSTICK}
    private WOBBLE_ARM_MODE waMode = WOBBLE_ARM_MODE.BUTTONS;
    private boolean bIgnoreStops = false;
    private int loop = 0;

    void setPoseStart() {
        // This code assumes Robot starts at a position as follows:
        // Align the left side of the robot with the INSIDE start line (TILE_1_FROM_ORIGIN in Y axis)
        // Robot Heading is pointing to +ve X-axis  (Ring Shooter Platform is facing the goals)
        // Robot back is touching the perimeter wall.
        globalPosition.setGlobalPosition(poseStart.getX(), poseStart.getY(), poseStart.getHeading());
        rrmdrive.setPoseEstimate(poseStart);

        // Enable this temporarily for Shooter Platform Tilt debugging
        //globalPosition.initGlobalPosition(ORIGIN, TILE_2_CENTER- UGoalRobot.ROBOT_SHOOTING_Y_OFFSET, ANGLE_POS_X_AXIS);
        //rrmdrive.setPoseEstimate(new Pose2d(ORIGIN, TILE_2_CENTER- UGoalRobot.ROBOT_SHOOTING_Y_OFFSET, ANGLE_POS_X_AXIS));
    }

    @Override
    public void runOpMode() {

        FieldUGoal.aColor = AllianceColor.BLUE;

        // Create main OpMode member objects initialize the hardware variables.
        robot = new UGoalRobot(hardwareMap,this);
        mcdrive = robot.getMCBdrive();
        rrmdrive = robot.getRRMdrive();
        driver = new UGoalTeleOpDriver(this, rrmdrive, mcdrive, robot);
        globalPosition = mcdrive.getOdometry();

        // set the starting Pose same as last saved Pose from the AUTO Opmode
        rrmdrive.setPoseEstimate(savedPose);
        globalPosition.setGlobalPosition(savedPose.getX(), savedPose.getY(), savedPose.getHeading());

        robot.composeTelemetry();
        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Waiting for Start");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // start the thread which handles driver controls on gamepad1
        driver.start();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (++loop % 10 == 0) {
                robot.setLED4RingsCount();
            }
            setup();
            intake();
            shootRings();
            wobblePickup();
            telemetry.update();
            idle();
        }

        // Reset the tilt angle of the shooting platform
        robot.tiltShooterPlatformMin();
        //Stop the thread
        globalPosition.stop();
        driver.stop();
        // all done, please exit

    }

    public void setup() {
        /*
         * Use special key combinations to toggle override controls
         */
        // Allow the lift stops to be overridden
        // This is necessary and useful when Robot is powered off with lift still raised high
        // and next power up initializes the lift bottom stop in that raised position
        if ((gamepad2.start) && (gamepad2.left_bumper)) {
            bIgnoreStops = false;
        }
        // Enforces the lift stops (top and bottom) as normal function
        if ((gamepad2.start) && (gamepad2.right_bumper)) {
            bIgnoreStops = true;
        }

        if ((gamepad2.start) && (gamepad2.x)) {
            robot.raiseIntakeAssembly();
            robot.resetWobblePickupArmEncoder();
            mcdrive.resetDriveEncoder();
            mcdrive.initIMU();
            rrmdrive.initIMU(); // both initIMU() do the same thing, but in future we may keep only 1 drive instance
            globalPosition.resetOdometryEncoder();
            // Ensure to set Pose after IMU initialization has been done
            setPoseStart();
        }
    }

    public void shootRings() {
        // Toggle the ring shooter flywheel motor when A is pressed
        if (gamepad2.a) {                               if (!gamepad2ADebounce) {
                if (robot.isShooterFlywheelRunning()) {
                    robot.stopShooterFlywheel();
                } else {
                    robot.runShooterFlywheel();
                }
            }
            gamepad2ADebounce = true;
        } else {
            gamepad2ADebounce = false;
        }
        // Shoot when B is pressed
        if (gamepad2.b) {
            //check if flywheel is running
            if (robot.isShooterFlywheelRunning() && robot.ringPusher.getPosition() > 0.975) {
                robot.shootRing();
            }
        }
        //auto aim for High Goal
        if (gamepad2.x && !gamepad2.start) {
            robot.tiltShooterPlatform(Target.HIGHGOAL);
        }
        //auto aim for Powershot
        if (gamepad2.y) {
            robot.tiltShooterPlatform(Target.POWERSHOT_1);
        }

        if (gamepad2.dpad_right) {
            robot.tiltShooterPlatformMin();
        }
        if (gamepad2.dpad_left) {
            robot.tiltShooterPlatformMax();
        }
    }

    public void intake() {

        double power;
        if (gamepad2.right_trigger > 0) { // intake is sucking the ring in to the robot
            power = gamepad2.right_trigger;
        } else if (gamepad2.left_trigger > 0) { // intake is ejecting the ring out of the robot
            power = -gamepad2.left_trigger;
        } else { // stop intake motor
            power = 0.0;
        }
        // Square the number but retain the sign to convert to logarithmic scale
        // scale the range to 0.30 <= abs(power) <= 1.0 and preserve the sign
        //power = Math.signum(power) * (0.25 + (0.75 * power * power)) * speedMultiplier;
        robot.runIntake(power);
        //telemetry.addData("Intake power ", "%.2f", power);
    }

    public void wobblePickup() {
        // Wobble finger
        if (gamepad2.right_bumper) {
            robot.wobbleFinger.setPosition(UGoalRobot.WOBBLE_FINGER_CLOSED);
        }
        if (gamepad2.left_bumper) {
            robot.wobbleFinger.setPosition(UGoalRobot.WOBBLE_FINGER_OPEN);
            //release the intake down to the ground. This is button overloading.
            robot.dropIntakeAssembly();
        }

        //error margin is to prevent the wobble arm from being unable to go to a certain spot
        if (gamepad2.dpad_up) { // operator trying to move wobble arm UP
             if (!gamepad2DpadDebounce){
                 waMode = WOBBLE_ARM_MODE.BUTTONS;
                 robot.moveWobbleArmUpwards();
             }
             gamepad2DpadDebounce = true;
        } else if (gamepad2.dpad_down) { // operator trying to move wobble arm DOWN
             if (!gamepad2DpadDebounce){
                 waMode = WOBBLE_ARM_MODE.BUTTONS;
                 robot.moveWobbleArmDownwards();
             }
             gamepad2DpadDebounce = true;
        } else {
            gamepad2DpadDebounce = false;
        }

        if (gamepad2.right_stick_y != 0) {

            //joystick gives a negative value when pushed up, we want the wobbelArm to go up when positive
            double power = -gamepad2.right_stick_y;
            // Square the number but retain the sign to convert to logarithmic scale
            // scale the range to 0.15 <= abs(power) <= 1.0 and preserve the sign
            power = Math.signum(power) * (0.15 + (0.85 * power * power));
            int pos = robot.wobblePickupArm.getCurrentPosition();

            if (waMode == WOBBLE_ARM_MODE.BUTTONS) {
                // if motor is in RUN_TO_POSITION mode then it resists free movement using joystick
                robot.wobblePickupArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                waMode = WOBBLE_ARM_MODE.JOYSTICK;
            }
            //if stops are being ignored then simply apply the joystick power to the motor
            if (bIgnoreStops) {
                telemetry.addData("Wobble ", "at %d, IGNORING STOPS", pos);
                robot.moveWobbleArm(power);
            }
            // move upwards direction but respect the stop to avoid mechanical damage
            else if (power > 0 && pos < UGoalRobot.WOBBLE_ARM_MAX) {
                telemetry.addData("Wobble Up ", "power %.2f | pos %d", power, pos);
                robot.moveWobbleArm(power);
            }
            // move downwards direction but respect the stop to avoid mechanical damage
            else if (power < 0 && pos > UGoalRobot.WOBBLE_ARM_DOWN) {
                telemetry.addData("Wobble Down ", "power %.2f | pos %d", power, pos);
                robot.moveWobbleArm(power);
            }
            // DO NOT remove the following line, we need to call stopWobbleArm() despite power is non-zero, when STOPS are reached
            else {
                robot.stopWobbleArm();
            }
        }
        // DO NOT remove the following line, we need to call stopWobbleArm() when user releases the joystick
        else if (waMode == WOBBLE_ARM_MODE.JOYSTICK) {
            robot.stopWobbleArm();
        }
    }
}