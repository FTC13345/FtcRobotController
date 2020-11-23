package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;

import org.firstinspires.ftc.teamcode.robot.MecabotMove;


@TeleOp(name = "UGoal TeleOp", group="QT")
public class UGoalTeleOp extends LinearOpMode {

    static final double TURN_FACTOR =   0.6;    // slow down turning speed
    private boolean     bIgnoreLiftStops = false;

    /* Declare OpMode members. */
    UGoalRobot robot;
    OdometryGlobalPosition globalPosition;

    // record position that we need to return to repeatedly
    double xpos, ypos, tpos;
    boolean autoDriving = false;
    double speedMultiplier = MecabotMove.DRIVE_SPEED_MAX;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables.
        robot = new UGoalRobot(hardwareMap, this);
        telemetry.addData(">", "Hardware initialized");

        globalPosition = robot.getPosition();

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Waiting for Start");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // This code assumes Robot starts at a position as follows:
        // Align the left side of the robot with the INSIDE start line (TILE_1_FROM_ORIGIN in Y axis)
        // Robot Heading is pointing to +ve X-axis  (Ring Shooter Platform is facing the goals)
        // Robot back is touching the perimeter wall.
        globalPosition.initGlobalPosition(-FieldUGoal.TILE_3_FROM_ORIGIN+robot.HALF_WIDTH, FieldUGoal.TILE_1_FROM_ORIGIN+robot.HALF_WIDTH, FieldUGoal.ANGLE_POS_X_AXIS);
        // Enable this temporarily for Shooter Platform Tilt debugging
        //globalPosition.initGlobalPosition(FieldUGoal.ORIGIN, FieldUGoal.TILE_2_CENTER-robot.ROBOT_SHOOTING_CURVE_OFFSET, FieldUGoal.ANGLE_POS_X_AXIS);
        robot.startOdometry();

        telemetry.addLine("Global Position ")
                .addData("X", "%2.2f", new Func<Double>() {
                    @Override public Double value() {
                        return globalPosition.getXinches();
                    }
                })
                .addData("Y", "%2.2f", new Func<Double>() {
                    @Override public Double value() {
                        return globalPosition.getYinches();
                    }
                })
                .addData("Angle", "%3.2f", new Func<Double>() {
                    @Override public Double value() {
                        return globalPosition.getOrientationDegrees();
                    }
                });
        telemetry.addLine("Odometry ")
                .addData("L", "%5.0f", new Func<Double>() {
                    @Override public Double value() {
                        return globalPosition.getVerticalLeftCount();
                    }
                })
                .addData("R", "%5.0f", new Func<Double>() {
                    @Override public Double value() {
                        return globalPosition.getVerticalRightCount();
                    }
                })
                .addData("X", "%5.0f", new Func<Double>() {
                    @Override public Double value() {
                        return globalPosition.getHorizontalCount();
                    }
                });
        telemetry.addLine("Move ")
                .addData("", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.getMovementStatus();
                    }
                });
        telemetry.addLine("Tilt ")
                .addData("Deg", "%.1f", new Func<Double>() {
                    @Override public Double value() {
                        return robot.shooterTiltAngleDesired;
                    }
                })
                .addData("Oval", "%.1f", new Func<Double>() {
                    @Override public Double value() {
                        return robot.ovalRotationDegrees;
                    }
                })
                .addData("TPos", "%4d", new Func<Integer>() {
                    @Override public Integer value() {
                        return robot.ovalRotationTicks;
                    }
                })
                .addData("CPos", "%4d", new Func<Integer>() {
                    @Override
                    public Integer value() {
                        return robot.angleMotor.getCurrentPosition();
                    }
                });


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            setup();
            autodrive();
            operdrive();
            intake();
            shootRings();
            wobblePickup();
            ringsLiftArmClaw();
            telemetry.update();
            idle();
        }

        // Reset the tilt angle of the shooting platform
        robot.resetShooterPlatform();
        //Stop the thread
        robot.stopOdometry();

        // all done, please exit

    }

    private void setup() {
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
/*
        // This block of code was used to record the position of robot on the field and
        // then later auto drive back to same location. Used in Skystone, no longer used in Ultimate Goal.
       if ((gamepad1.x) && (!gamepad2.x)) {
            xpos = globalPosition.getXinches();
            ypos = globalPosition.getYinches();
            tpos = globalPosition.getOrientationDegrees();
            telemetry.addData("Locked Position", "X %2.2f | Y %2.2f | Angle %3.2f", xpos, ypos, tpos);
        }
        if ((gamepad1.y) && (!gamepad2.y)){
            robot.setDirectionReverse();
            autoDriving = true;
        }
 */
        if (gamepad1.start) {
            // Toggle which face of the Robot is front for driving
            if (gamepad1.right_bumper) {
                robot.setDirectionForward();
            } else if (gamepad1.left_bumper) {
                robot.setDirectionReverse();
            }
            // reset the odometry encoders to zero
            else if (gamepad1.x) {
                robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                globalPosition.initGlobalPosition(-FieldUGoal.TILE_3_FROM_ORIGIN+robot.HALF_WIDTH, FieldUGoal.TILE_1_FROM_ORIGIN+robot.HALF_WIDTH, FieldUGoal.ANGLE_POS_X_AXIS);
            }
        }
        else { // !gamepad1.start --> which means bumper buttons pressed alone
            //update speedMultiplier for FAST or SLOW driving
            if (gamepad1.right_bumper) {
                speedMultiplier = MecabotMove.DRIVE_SPEED_MAX;
                robot.setFastBlue();
                // as a dual action of this button stop autodriving
                autoDriving = false;
            }
            else if (gamepad1.left_bumper) {
                speedMultiplier = MecabotMove.DRIVE_SPEED_DEFAULT;
                robot.setSlowBlue();
                // as a dual action of this button stop autodriving
                autoDriving = false;
            }
        }
    }

    public void autodrive() {
        if (autoDriving) {
            telemetry.addData("Driving Towards", "X %2.2f | Y %2.2f | Angle %3.2f", xpos, ypos, tpos);
            double distance = robot.goTowardsPosition(xpos, ypos, MecabotMove.DRIVE_SPEED_DEFAULT, true);
            if (distance < MecabotMove.DIST_MARGIN) { // we have reached
                autoDriving = false;
            }
        }
    }

    public void operdrive() {
        // if joystick is inactive brakes will be applied during autodrive() therefore don't go in
        if (autoDriving) {
            return;
        }

        double power, turn;

        // square the joystick values to change from linear to logarithmic scale
        // this allows more movement of joystick for less movement of robot, thus more precision at lower speeds
        // at the expense of some loss of precision at higher speeds, where it is not required.

        // when we want to move sideways (MECANUM)
        if (gamepad1.left_trigger > 0) {
            power = -gamepad1.left_trigger; // negative power to move LEFT
            // Square the number but retain the sign to convert to logarithmic scale
            // scale the range to 0.30 <= abs(power) <= 1.0 and preserve the sign
            power = Math.signum(power) * (0.25 + (0.75 * power * power)) * speedMultiplier;
            robot.driveMecanum(power);
            telemetry.addData("Mecanum Left ", "%.2f", power);
        }
        else if (gamepad1.right_trigger > 0) {
            power = gamepad1.right_trigger; // positive power to move RIGHT
            // Square the number but retain the sign to convert to logarithmic scale
            // scale the range to 0.30 <= abs(power) <= 1.0 and preserve the sign
            power = Math.signum(power) * (0.25 + (0.75 * power * power)) * speedMultiplier;
            robot.driveMecanum(power);
            telemetry.addData("Mecanum Right  ", "%.2f", power);
        }
        // normal tank movement
        else {  // when joystick is inactive, this applies brakes, be careful to avoid during autodrive
            // forward press on joystick is negative, backward press (towards human) is positive
            // right press on joystick is positive value, left press is negative value
            // reverse sign of joystick values to match the expected sign in driveTank() method.
            power = -gamepad1.left_stick_y;
            turn = -gamepad1.right_stick_x;

            // Square the number but retain the sign to convert to logarithmic scale
            // scale the range to 0.25 <= abs(power) <= 1.0 and preserve the sign
            power = Math.signum(power) * (0.2 + (0.8 * power * power));
            // OR
            // use this to scale the range without squaring the power value
            //power = Math.signum(power) * (0.25 + (0.75 * Math.abs(power)));
            // OR
            // use this to square the power while preserving the sign, without scaling the range
            //power *= Math.abs(power);

            // similarly for turn power, except also slow down by TURN_FACTOR
            turn = Math.signum(turn) * (0.1 + (TURN_FACTOR * turn * turn));

            robot.driveTank(power, turn);
            //telemetry.addData("Tank Power", "Drive=%.2f Turn=%.2f", power, turn);
        }
    }
    public void shootRings(){
       // Toggle the ring shooter flywheel motor when A is pressed
        if (gamepad2.a) {
            if (robot.isShooterFlywheelRunning()) {
                robot.stopShooterFlywheel();
            }
            else {
                robot.runShooterFlywheel();
            }
        }
        // Shoot when B is pressed
        else if (gamepad2.b) {
            robot.shootRing();

        }
        //auto aim for High Goal
        else if (gamepad2.x) {
            double targetAngle = robot.calculateRobotHeadingToShoot(FieldUGoal.GOALX, FieldUGoal.GOALY);
            telemetry.addData("Rotate to Angle ", "%2.2f for High Goal", targetAngle);
            robot.rotateToHeading(targetAngle);
            robot.tiltShooterPlatform(FieldUGoal.GOALX, FieldUGoal.GOALY, FieldUGoal.HIGH_GOAL_HEIGHT);
        }
        //auto aim for Powershot
        else if (gamepad2.y) {
            double targetAngle = robot.calculateRobotHeadingToShoot(FieldUGoal.GOALX, FieldUGoal.POWERSHOT_1_Y);
            telemetry.addData("Rotate to Angle ", "%2.2f for PowerShot 1", targetAngle);
            robot.rotateToHeading(targetAngle);
            robot.tiltShooterPlatform(FieldUGoal.GOALX, FieldUGoal.POWERSHOT_1_Y, FieldUGoal.POWER_SHOT_HEIGHT);
        }

        if (gamepad1.a) {
            robot.odometryRotateToHeading(FieldUGoal.ANGLE_POS_X_AXIS, MecabotMove.ROTATE_SPEED_DEFAULT, MecabotMove.TIMEOUT_ROTATE);
        }
        else if (gamepad1.b) {
            robot.gyroRotateToHeading(FieldUGoal.ANGLE_POS_X_AXIS, MecabotMove.ROTATE_SPEED_DEFAULT, MecabotMove.TIMEOUT_ROTATE);
        }
        else if (gamepad1.x) {
            robot.tiltShooterPlatform(robot.SHOOTER_TILT_ANGLE_MIN);
        }
        else if (gamepad1.y) {
            robot.tiltShooterPlatform(robot.SHOOTER_TILT_ANGLE_MAX);
        }
        if (gamepad1.start) {
            if (gamepad1.dpad_up) {
                robot.driveToShootHighGoal(FieldUGoal.AllianceColor.BLUE);
            }
            if (gamepad1.dpad_left) {
                robot.driveToShootPowerShot1(FieldUGoal.AllianceColor.BLUE);
            }
            if (gamepad1.dpad_down) {
                robot.driveToShootPowerShot2(FieldUGoal.AllianceColor.BLUE);
            }
            if (gamepad1.dpad_right) {
                robot.driveToShootPowerShot3(FieldUGoal.AllianceColor.BLUE);
            }
        }
    }

    public void intake() {

        double power = 0;
        if (gamepad2.right_trigger > 0) { // intake is sucking the ring in to the robot
            power = gamepad2.right_trigger;
        }
        else if (gamepad2.left_trigger > 0) { // intake is ejecting the ring out of the robot
            power = -gamepad2.left_trigger;
        }
        else { // stop intake motor
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
            //release the intake
            robot.releaseIntake();
        }
        if (gamepad2.left_bumper) {
            robot.wobbleFinger.setPosition(UGoalRobot.WOBBLE_FINGER_OPEN);
        }

        // Wobble arm
        // Note that when joystick is inactive, this applies brakes
        // forward press on joystick is negative, backward press (towards human) is positive
        // reverse sign of joystick values to use positive values to lift up the wobble arm
        double power = -gamepad2.left_stick_y;
        // Square the number but retain the sign to convert to logarithmic scale
        // scale the range to 0.1 <= abs(power) <= 0.5 and preserve the sign
        power = Math.signum(power) * ((power * power));
        int pos = robot.wobblePickupArm.getCurrentPosition();
        if ((power>0 && pos<robot.WOBBLE_ARM_UP) || (power<0 && pos>robot.WOBBLE_ARM_DOWN)) {
            robot.wobblePickupArm.setPower(power);
            telemetry.addData("Wobble Arm ", "power %.2f | pos %d", power, pos);
        }
    }

    // lift for the claw putting loaded rings onto the wobble goal, right side
    public void ringsLiftArmClaw(){

        // Lift Arm and Claw
        if (gamepad2.dpad_left) {
            robot.rotateClawOutside();
        } else if (gamepad2.dpad_right) {
            robot.rotateClawInside();
        } else if (gamepad2.dpad_down) {
            robot.grabRingsWithClaw();
        } else if (gamepad2.dpad_up) {
            robot.releaseRingsWithClaw();
        }

        if (gamepad2.right_stick_y != 0){
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
                robot.liftMotor.setPower(power);
            }
            // move lift upwards direction but respect the stop to avoid breaking string
            else if (power > 0 && pos < robot.LIFT_TOP) {
                telemetry.addData("Lift Up ", "power %.2f | pos %d", power, pos);
                robot.liftMotor.setPower(power);
            }
            // move lift downwards direction but respect the stop to avoid winding string in opposite direction on the spool
            else if (power < 0 && pos > robot.LIFT_BOTTOM) {
                telemetry.addData("Lift Down ", "power %.2f | pos %d", power, pos);
                robot.liftMotor.setPower(power);
            }
            // DO NOT remove the following line, we need to call stopLift() despite power is non-zero, when STOPS are reached
            else {
                robot.stopLift();
            }
        }
        // DO NOT remove the following line, we need to call stopLift() when user releases the joystick
        else{
            robot.stopLift();
        }
    }

}
