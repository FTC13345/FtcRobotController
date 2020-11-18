package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;

import org.firstinspires.ftc.teamcode.robot.MecabotMove;


@TeleOp(name = "UGoal TeleOp", group="QT")
public class UGoalTeleOp extends LinearOpMode {

    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double TURN_FACTOR =   0.6;    // slow down turning speed

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
        //release the intake
        robot.releaseIntake();

        // This code assumes Robot starts at a position as follows:
        // Align the left side of the robot with the INSIDE start line (TILE_1_FROM_ORIGIN in Y axis)
        // Robot Heading is pointing to +ve X-axis  (Ring Shooter Platform is facing the goals)
        // Robot back is touching the perimeter wall.
        globalPosition.initGlobalPosition(-FieldUGoal.TILE_3_FROM_ORIGIN + FieldUGoal.ROBOT_RADIUS, FieldUGoal.TILE_1_FROM_ORIGIN - FieldUGoal.ROBOT_RADIUS, FieldUGoal.ANGLE_POS_X_AXIS);
        // Enable this temporarily for Shooter Platform Tilt debugging
        // globalPosition.initGlobalPosition(FieldUGoal.BEHIND_LAUNCH_LINE, FieldUGoal.TILE_2_CENTER, 0.0);
        robot.startOdometry();

        telemetry.addLine("Global Position ")
                .addData("X", "%3.2f", new Func<Double>() {
                    @Override public Double value() {
                        return globalPosition.getXinches();
                    }
                })
                .addData("Y", "%2.2f", new Func<Double>() {
                    @Override public Double value() {
                        return globalPosition.getYinches();
                    }
                })
                .addData("Angle", "%4.2f", new Func<Double>() {
                    @Override public Double value() {
                        return globalPosition.getOrientationDegrees();
                    }
                });
        telemetry.addLine("Odometry ")
                .addData("L", "%4.2f", new Func<Double>() {
                    @Override public Double value() {
                        return globalPosition.getVerticalLeftCount();
                    }
                })
                .addData("R", "%4.2f", new Func<Double>() {
                    @Override public Double value() {
                        return globalPosition.getVerticalRightCount();
                    }
                })
                .addData("X", "%4.2f", new Func<Double>() {
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
        telemetry.addLine("Shooter ")
                .addData("Tilt", "%.2f", new Func<Double>() {
                    @Override public Double value() {
                        return robot.shooterTiltAngleDesired;
                    }
                })
                .addData("Oval", "%.2f", new Func<Double>() {
                    @Override public Double value() {
                        return robot.ovalRotationDegrees;
                    }
                })
                .addData("TPos", "%3d", new Func<Integer>() {
                    @Override public Integer value() {
                        return robot.ovalRotationTicks;
                    }
                })
                .addData("CPos", "%3d", new Func<Integer>() {
                    @Override
                    public Integer value() {
                        return robot.leftFrontDrive.getCurrentPosition();
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
            //calibration methods
            //runPickupWobble();

            runLift();
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
        // Toggle which face of the Robot is front for driving
        if ((gamepad1.dpad_up) || (gamepad1.dpad_right)) {  // dpad_up or dpad_right means green INTAKE wheels is front of robot
            robot.setDirectionForward();
        } else if ((gamepad1.dpad_down) || (gamepad1.dpad_left)) {
            robot.setDirectionReverse(); // dpad_down or dpad_left means REVERSE direction, ring shooter is front of robot
        }
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
        //update speedMultiplier
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
        if (gamepad2.b) {
            robot.shootRing();

        }

        //auto aim for High Goal
        if (gamepad2.x) {
            double heading = robot.calculateRobotHeadingToShoot(globalPosition.getXinches(), globalPosition.getYinches(), FieldUGoal.GOALX, FieldUGoal.GOALY);
            robot.odometryRotateToHeading(heading, 0.5, 5, true);
            telemetry.addData("Rotate to Target ", "%2.2f", Math.toDegrees(heading));

            robot.tiltShooterPlatform(FieldUGoal.GOALX, FieldUGoal.GOALY, FieldUGoal.HIGH_GOAL_HEIGHT);
        }

        //auto aim for Powershot
        // Assumption that human driver will set robot heading, we just need to shoot assuming power shot is straight ahead
        if (gamepad2.y) {
            robot.tiltShooterPlatform(FieldUGoal.GOALX, globalPosition.getYinches(), FieldUGoal.POWER_SHOT_HEIGHT);
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

    //calibration method, remove later
    public void runPickupWobble(){
        if (gamepad2.dpad_up) {
            robot.pickUpWobble(MecabotMove.DRIVE_SPEED_DEFAULT);
        }
        // no else because pickupWobble already resets by itself

    }

    public void wobblePickup() {
        // Wobble finger
        if (gamepad2.right_bumper) {
            robot.wobbleFinger.setPosition(UGoalRobot.WOBBLE_FINGER_CLOSED);
        }
        if (gamepad2.left_bumper) {
            robot.wobbleFinger.setPosition(UGoalRobot.WOBBLE_FINGER_OPEN);
        }

        // Wobble arm
        // Note that when joystick is inactive, this applies brakes
        // forward press on joystick is negative, backward press (towards human) is positive
        // reverse sign of joystick values to use positive values to lift up the wobble arm
        double power = -gamepad2.right_stick_y;
        // Square the number but retain the sign to convert to logarithmic scale
        // scale the range to 0.0 <= abs(power) <= 0.3 and preserve the sign
        power = Math.signum(power) * (0.3 * power * power);
        robot.wobbleFingerArm.setPower(power);
        //telemetry.addData("Wobble Arm ", "power %.2f | pos %d", power, robot.wobbleFingerArm.getCurrentPosition());
    }

    // lift for the claw putting loaded rings onto the wobble goal, right side
    public void runLift(){
        if (gamepad2.left_stick_y != 0){
            //joystick gives a negative value when pushed up, we want the lift to go up when positive
            //temporarily uncomment below and force slow speed while testing for stops
            //double power = -gamepad2.left_stick_y;
            double power = 0;
            if (-gamepad2.left_stick_y>0){
                //if joystick pushed up, lift goes up
                power = MecabotMove.DRIVE_SPEED_SLOW;
            }
            else{
                //if joystick pushed down, lift goes down
                power = -MecabotMove.DRIVE_SPEED_SLOW;
            }

            // Square the number but retain the sign to convert to logarithmic scale
            // scale the range to 0.15 <= abs(power) <= 1.0 and preserve the sign
            power = Math.signum(power) * (0.15 + (0.85 * power * power));
            int pos = robot.liftMotor.getCurrentPosition();

            //temporary for finding lift stop values
            if (power>0){
                robot.liftMotor.setPower(power);
                pos = robot.liftMotor.getCurrentPosition();
                telemetry.addData("Lift Up", "%.2f", power, pos);
            }
            else if (power<0){
                robot.liftMotor.setPower(power);
                pos = robot.liftMotor.getCurrentPosition();
                telemetry.addData("Lift down", "%.2f", power, pos);
            }
            else{
                robot.stopLift();
            }

            //uncomment once we have values for stop
            /* if lift stops are being ignored then simply apply the joystick power to the motor
            //            if (bIgnoreLiftStops) {
            //                robot.liftMotor.setPower(power);
            //                telemetry.addData("LIFT ", "at %3d, IGNORING STOPS", pos);
            //            }
            //            // move lift upwards direction but respect the stop to avoid breaking string
            //            else if (power > 0 && pos < robot.LIFT_TOP) {
            //                robot.liftMotor.setPower(power);
            //                telemetry.addData("Lift Up", "%.2f", power);
            //            }
            //            // move lift downwards direction but respect the stop to avoid winding string in opposite direction on the spool
            //            else if (power < 0 && pos > robot.LIFT_BOTTOM) {
            //                robot.liftMotor.setPower(power);
            //                telemetry.addData("Lift Down", "%.2f", power);
            //            }
            //            else {
            //                robot.stopLift();
            //            }
            */
        }
        else{
            robot.stopLift();
        }
    }

    /*public void runPutRingsOnWobble(){
        if (){//what controls to use?
            robot.putRingsOnWobble();
        }
    }
    public void runPickupWobble(){
        if(){//what controls to use?
            robot.pickUpWobble();
        }
    }

    public void runPlaceWobble(){
        if() {//what controls to use?
            robot.placeWobble();
        }
    }
    */



}
