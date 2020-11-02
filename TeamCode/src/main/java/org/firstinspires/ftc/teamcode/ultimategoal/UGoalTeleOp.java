package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        nav.startOdometry();

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

}
