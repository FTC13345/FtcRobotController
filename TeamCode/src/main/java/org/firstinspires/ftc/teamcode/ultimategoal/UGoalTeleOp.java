package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;

import org.firstinspires.ftc.teamcode.robot.MecabotMove;


@TeleOp(name = "UGoal TeleOp", group="QT")
public class UGoalTeleOp extends LinearOpMode {

    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double TURN_FACTOR =   0.6;    // slow down turning speed

    /* Declare OpMode members. */
    UGoalRobot robot;
    MecabotMove nav;
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

        nav = (MecabotMove)robot;
        globalPosition = nav.getPosition();

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Waiting for Start");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //release the intake
        robot.releaseIntake();
        nav.startOdometry();

        globalPosition.initGlobalPosition(-FieldUGoal.TILE_3_FROM_ORIGIN + FieldUGoal.ROBOT_RADIUS, FieldUGoal.TILE_2_FROM_ORIGIN - FieldUGoal.ROBOT_RADIUS, 90);

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
                        return nav.getMovementStatus();
                    }
                });
        telemetry.addLine("Odometry Inches:")
                .addData("X", "%4.2f", new Func<Double>() {
                    @Override public Double value() {
                        return globalPosition.getXinches();
                    }
                })
                .addData("Y", "%4.2f", new Func<Double>() {
                    @Override public Double value() {
                        return globalPosition.getYinches();
                    }
                });

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            setup();
            autodrive();
            operdrive();
            intake();
            shoot();
            wobble();

            runLift();
            telemetry.update();
            idle();
        }

        //Stop the thread
        nav.stopOdometry();

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
            double distance = nav.goTowardsPosition(xpos, ypos, MecabotMove.DRIVE_SPEED_DEFAULT, true);
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
            telemetry.addData("Tank Power", "Drive=%.2f Turn=%.2f", power, turn);
        }
    }
    public void shoot(){
        // Toggle the launcher motor when A is pressed
        if (gamepad2.a) {
            if (robot.isLauncherMotorRunning()) {
                robot.stopLauncherMotor();
            }
            else {
                robot.runLauncherMotor();
            }
        }

        // Shoot when B is pressed
        if (gamepad2.b) {
            robot.shootRing();

        }
        //auto aim for High Goal
        if (gamepad2.y) {
            robot.tiltLaunchPlatform(FieldUGoal.HIGH_GOAL_HEIGHT);
        }

        //auto aim for Powershot
        if (gamepad2.x) {
            robot.tiltLaunchPlatform(FieldUGoal.POWER_SHOT_HEIGHT);
        }
    }

    public void intake(){
        double power = -gamepad2.right_stick_y;
        if (power > 0) {
            robot.runIntake(power);
            telemetry.addData("Intake Power ", "%.2f", power);
        }
        else {
            robot.stopIntake();
        }
    }

    public void wobble() {
        // Wobble finger
        if (gamepad2.right_bumper) {
            robot.wobbleFinger.setPosition(Servo.MAX_POSITION);
        }
        if (gamepad2.left_bumper) {
            robot.wobbleFinger.setPosition(Servo.MIN_POSITION);
        }

        double power = 0;
        if (gamepad2.right_trigger > 0) {
            power = gamepad2.right_trigger;
            robot.wobbleFingerArm.setPower(power);
            telemetry.addData("Wobble Arm ", "%.2f", power);
        }
        else if (gamepad2.left_trigger > 0) {
            power = -gamepad2.left_trigger;
            robot.wobbleFingerArm.setPower(power);
            telemetry.addData("Wobble Arm ", "%.2f", power);
        }

        // Wobble claw
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
