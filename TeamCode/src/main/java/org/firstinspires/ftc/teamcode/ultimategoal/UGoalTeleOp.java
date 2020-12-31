package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.MecabotDrive;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TeleOpDriver;

import static org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.*;


@TeleOp(name = "UGoal TeleOp", group="QT")
public class UGoalTeleOp extends LinearOpMode {

    private boolean bIgnoreLiftStops = false;

    /* Declare OpMode members. */
    RRMecanumDrive rrmdrive;
    MecabotDrive mcdrive;
    UGoalRobot robot;
    OdometryGlobalPosition globalPosition;
    TeleOpDriver driver;
    Telemetry drvrTelemetry;
    Telemetry dashTelemetry;

    //Button debounce
    private boolean gamepad2ADebounce = false;
    private boolean gamepad2DpadDebounce = false;

    //Wobble Pos tracker
    int wobblePos = 0;

    void setOdometryStartingPosition() {
        // This code assumes Robot starts at a position as follows:
        // Align the left side of the robot with the INSIDE start line (TILE_1_FROM_ORIGIN in Y axis)
        // Robot Heading is pointing to +ve X-axis  (Ring Shooter Platform is facing the goals)
        // Robot back is touching the perimeter wall.
        globalPosition.initGlobalPosition(-TILE_3_FROM_ORIGIN + ROBOT_RADIUS, TILE_1_FROM_ORIGIN + ROBOT_RADIUS, ANGLE_POS_X_AXIS);
        rrmdrive.setPoseEstimate(new Pose2d(-TILE_3_FROM_ORIGIN + ROBOT_RADIUS, TILE_1_FROM_ORIGIN + ROBOT_RADIUS, ANGLE_POS_X_AXIS));

        // Enable this temporarily for Shooter Platform Tilt debugging
        //globalPosition.initGlobalPosition(ORIGIN, TILE_2_CENTER- UGoalRobot.ROBOT_SHOOTING_CURVE_OFFSET, ANGLE_POS_X_AXIS);
        //rrmdrive.setPoseEstimate(new Pose2d(ORIGIN, TILE_2_CENTER- UGoalRobot.ROBOT_SHOOTING_CURVE_OFFSET, ANGLE_POS_X_AXIS));
    }

    @Override
    public void runOpMode() {

        FieldUGoal.aColor = AllianceColor.BLUE;

        // Redirect telemetry printouts to both Driver Station and FTC Dashboard
        dashTelemetry = FtcDashboard.getInstance().getTelemetry();
        drvrTelemetry = telemetry;
        //telemetry = new MultipleTelemetry(drvrTelemetry, dashTelemetry);

        // Create main OpMode member objects initialize the hardware variables.
        rrmdrive = new RRMecanumDrive(hardwareMap);
        robot = new UGoalRobot(hardwareMap, this);
        mcdrive = robot.getDrive();
        driver = new TeleOpDriver(this, rrmdrive, mcdrive);

        globalPosition = mcdrive.getOdometry();
        setOdometryStartingPosition();

        setupTelemetry();
        telemetry.addData(">", "Hardware initialized");
        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Waiting for Start");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // start the thread which handles driver controls on gamepad1
        driver.start();

        // set starting position again, sometimes players move the robot between init() and start()
        setOdometryStartingPosition();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            setup();
            intake();
            shootRings();
            wobblePickup();
            // Disabled because the ringsLift has been removed in Robot hardware,
            // it does not earn enough points according to game strategy, same time can be used wisely
            // ringsLiftArmClaw();
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
            bIgnoreLiftStops = false;
        }
        // Enforces the lift stops (top and bottom) as normal function
        if ((gamepad2.start) && (gamepad2.right_bumper)) {
            bIgnoreLiftStops = true;
        }

        if ((gamepad2.start) && (gamepad2.x)) {
            robot.raiseIntakeAssembly();
            robot.resetWobblePickupArmEncoder();
            mcdrive.resetDriveEncoder();
            globalPosition.resetOdometryEncoder();
            setOdometryStartingPosition();
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
            robot.shootRing();

        }
        //auto aim for High Goal
        if (gamepad2.x && !gamepad2.start) {
// TEMPORARY because odometry is not working, do not do rotation towards Goals, driver will do manually using other controls
            double targetAngle = robot.calculateRobotHeadingToShoot(GOALX, GOALY);
            telemetry.addData("Rotate to Angle ", "%2.2f for High Goal", targetAngle);
            mcdrive.rotateToHeading(targetAngle);

            robot.tiltShooterPlatform(GOALX, GOALY, HIGH_GOAL_HEIGHT);
        }
        //auto aim for Powershot
        if (gamepad2.y) {
// TEMPORARY because odometry is not working, do not do rotation towards Goals, driver will do manually using other controls
//            double targetAngle = robot.calculateRobotHeadingToShoot(GOALX, POWERSHOT_1_Y);
//            telemetry.addData("Rotate to Angle ", "%2.2f for PowerShot 1", targetAngle);
//            robot.rotateToHeading(targetAngle);

            robot.tiltShooterPlatform(GOALX, POWERSHOT_1_Y, POWER_SHOT_HEIGHT);
        }

        if (gamepad2.dpad_right) {
            robot.tiltShooterPlatformMin();
        }
        if (gamepad2.dpad_left) {
            robot.tiltShooterPlatformMax();
        }
        if (gamepad1.start) {
            if (gamepad1.dpad_up) {
                robot.driveToShootHighGoal();
            }
            if (gamepad1.dpad_left) {
                robot.driveToShootPowerShot1();
            }
            if (gamepad1.dpad_down) {
                robot.driveToShootPowerShot2();
            }
            if (gamepad1.dpad_right) {
                robot.driveToShootPowerShot3();
            }
        }
    }

    public void intake() {

        double power = 0;
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
            //release the intake down to the ground. This is button overloading.
            robot.dropIntakeAssembly();
        }
        if (gamepad2.left_bumper) {
            robot.wobbleFinger.setPosition(UGoalRobot.WOBBLE_FINGER_OPEN);
        }

        if (gamepad2.dpad_up) { // operator trying to move wobble arm UP
             if (!gamepad2DpadDebounce && wobblePos < UGoalRobot.WOBBLE_ARM_POS.length - 1){
                 robot.goToWobblePos(++wobblePos);
             }
             gamepad2DpadDebounce = true;
        } else if (gamepad2.dpad_down) { // operator trying to move wobble arm DOWN
            if (!gamepad2DpadDebounce && wobblePos > 0){
                robot.goToWobblePos(--wobblePos);
            }
            gamepad2DpadDebounce = true;
        } else {
            gamepad2DpadDebounce = false;
        }
    }

    // lift for the claw putting loaded rings onto the wobble goal, right side
    public void ringsLiftArmClaw() {

        if (gamepad2.right_stick_y != 0) {
            //joystick gives a negative value when pushed up, we want the lift to go up when positive
            double power = -gamepad2.right_stick_y;
            // Square the number but retain the sign to convert to logarithmic scale
            // scale the range to 0.15 <= abs(power) <= 1.0 and preserve the sign
            power = Math.signum(power) * (0.15 + (0.85 * power * power));
            int pos = robot.getLiftCurrentPosition();

//            //temporarily uncomment below and force slow speed while testing for stops
//            //Note that if motor direction is reversed, then the LIFT_TOP and LIFT_BOTTOM will have wrong sign
//            // and the lift will not move in either direction, therefore we also need to disable the LIFT TOP and BOTTOM Stops
//            power = Math.signum(power) * robot.DRIVE_SPEED_SLOW;
//            bIgnoreLiftStops = true;

            //if lift stops are being ignored then simply apply the joystick power to the motor
            if (bIgnoreLiftStops) {
                telemetry.addData("LIFT ", "at %3d, IGNORING STOPS", pos);
                robot.runLift(power);
            }
            // move lift upwards direction but respect the stop to avoid breaking string
            else if (power > 0 && pos < robot.LIFT_TOP) {
                telemetry.addData("Lift Up ", "power %.2f | pos %d", power, pos);
                robot.runLift(power);
            }
            // move lift downwards direction but respect the stop to avoid winding string in opposite direction on the spool
            else if (power < 0 && pos > robot.LIFT_BOTTOM) {
                telemetry.addData("Lift Down ", "power %.2f | pos %d", power, pos);
                robot.runLift(power);
            }
            // DO NOT remove the following line, we need to call stopLift() despite power is non-zero, when STOPS are reached
            else {
                robot.stopLift();
            }
        }
        // DO NOT remove the following line, we need to call stopLift() when user releases the joystick
        else {
            robot.stopLift();
        }
    }

    public void setupTelemetry() {
        drvrTelemetry.addLine("Runner Position ")
                .addData("X", "%2.2f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return rrmdrive.getPoseEstimate().getX();
                    }
                })
                .addData("Y", "%2.2f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return rrmdrive.getPoseEstimate().getY();
                    }
                })
                .addData("Head", "%3.2f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return Math.toDegrees(rrmdrive.getPoseEstimate().getHeading());
                    }
                });
        drvrTelemetry.addLine("Global Position ")
                .addData("X", "%2.2f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return globalPosition.getXinches();
                    }
                })
                .addData("Y", "%2.2f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return globalPosition.getYinches();
                    }
                })
                .addData("Head", "%3.2f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return globalPosition.getOrientationDegrees();
                    }
                });
        drvrTelemetry.addLine("Odometry ")
                .addData("L", "%5.0f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return globalPosition.getVerticalLeftCount();
                    }
                })
                .addData("R", "%5.0f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return globalPosition.getVerticalRightCount();
                    }
                })
                .addData("X", "%5.0f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return globalPosition.getHorizontalCount();
                    }
                });
        drvrTelemetry.addLine("Drivetrain ")
                .addData("LF", "%5d", new Func<Integer>() {
                    @Override
                    public Integer value() {
                        return mcdrive.leftFrontDrive.getCurrentPosition();
                    }
                })
                .addData("LB", "%5d", new Func<Integer>() {
                    @Override
                    public Integer value() {
                        return mcdrive.leftBackDrive.getCurrentPosition();
                    }
                })
                .addData("RF", "%5d", new Func<Integer>() {
                    @Override
                    public Integer value() {
                        return mcdrive.rightFrontDrive.getCurrentPosition();
                    }
                })
                .addData("RB", "%5d", new Func<Integer>() {
                    @Override
                    public Integer value() {
                        return mcdrive.rightBackDrive.getCurrentPosition();
                    }
                });
        drvrTelemetry.addLine("Tilt ")
                .addData("Angle", "%.1f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return robot.shooterPlatformTiltAngle;
                    }
                })
                .addData("Pos", "%.2f",  new Func<Double>() {
                    @Override
                    public Double value() {
                        return robot.angleServo.getPosition();
                    }
                })
                .addData("Wobble Arm", "%3d", new Func<Integer>() {
                    @Override
                    public Integer value() {
                        return robot.wobblePickupArm.getCurrentPosition();
                    }
                });
        drvrTelemetry.addLine("Move ")
                .addData("", new Func<String>() {
                    @Override
                    public String value() {
                        return mcdrive.getMovementStatus();
                    }
                });
        drvrTelemetry.update();
    }

}