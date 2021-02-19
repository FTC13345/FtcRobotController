package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class TeleOpDriver implements Runnable {

    static final double TURN_FACTOR = 0.6;    // slow down turning speed

    // member variables for state
    protected LinearOpMode  myOpMode;       // Access to the OpMode object
    protected RRMecanumDrive rrmdrive;
    protected MecabotDrive  mcdrive;          // Access to the Robot hardware
    protected Gamepad       gamepad1;
    protected Telemetry     telemetry;

    protected Thread        myThread;
    protected boolean       isRunning = false;
    protected double        speedMultiplier = MecabotDrive.DRIVE_SPEED_MAX;

    boolean autoDriving = false;

    public abstract void  driveGameTeleOp();
    public abstract void  driveGameAuto();

    /* Constructor */
    public TeleOpDriver(LinearOpMode opMode, RRMecanumDrive rrmdrive, MecabotDrive mcdrive) {
        // Save reference to OpMode and Hardware map
        myOpMode = opMode;
        gamepad1 = opMode.gamepad1;
        telemetry = opMode.telemetry;
        this.rrmdrive = rrmdrive;
        this.mcdrive = mcdrive;
    }

    public void start() {

        isRunning = true;
        myThread = new Thread(this);
        myThread.start();
    }

    public void stop() {
        isRunning = false;
        myThread = null;
    }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning && myOpMode.opModeIsActive()) {
            setup();
            driveTeleOp();
            driveGameTeleOp();
            driveGameAuto();
            driveAuto();
            telemetry.update();
            myOpMode.idle();
        }
    }

    public void setup() {
        if (gamepad1.start) {
            // Toggle which face of the Robot is front for driving
            if (gamepad1.right_bumper) {
                speedMultiplier = MecabotDrive.DRIVE_SPEED_MAX;
                mcdrive.setFastBlue();
            } else if (gamepad1.left_bumper) {
                speedMultiplier = MecabotDrive.DRIVE_SPEED_DEFAULT;
                mcdrive.setSlowBlue();
            }
        }
        else { // !gamepad1.start --> which means bumper buttons pressed alone
            //update speedMultiplier for FAST or SLOW driving
            if (gamepad1.right_bumper) {
                mcdrive.setDirectionForward();
                // as a dual action of this button stop autodriving
                autoDriving = false;
            }
            else if (gamepad1.left_bumper) {
                mcdrive.setDirectionReverse();
                // as a dual action of this button stop autodriving
                autoDriving = false;
            }
        }
    }

    public void driveTeleOp() {
        // if joystick is inactive brakes will be applied during autodrive() therefore don't go in
        if (autoDriving) {
            return;
        }

        // square the joystick values to change from linear to logarithmic scale
        // this allows more movement of joystick for less movement of robot, thus more precision at lower speeds
        // at the expense of some loss of precision at higher speeds, where it is not required.
        double power = -gamepad1.left_stick_y;
        // Square the number but retain the sign to convert to logarithmic scale
        // scale the range to 0.2 <= abs(power) <= 1.0 and preserve the sign
        if (power != 0) {
            power = Math.signum(power) * (0.2 + (0.8 * power * power)) * speedMultiplier;
        }
        // OR
        // use this to scale the range without squaring the power value
        //power = Math.signum(power) * (0.25 + (0.75 * Math.abs(power)));
        // OR
        // use this to square the power while preserving the sign, without scaling the range
        //power *= Math.abs(power);

        double strafe = (gamepad1.left_trigger > 0) ? +gamepad1.left_trigger : -gamepad1.right_trigger;
        if (strafe != 0) {
            strafe = Math.signum(strafe) * (0.2 + (0.8 * strafe * strafe)) * speedMultiplier;
        }

        // similarly for turn power, except also slow down by TURN_FACTOR
        double turn = -gamepad1.right_stick_x;
        if (turn != 0) {
            turn = Math.signum(turn) * (0.1 + (TURN_FACTOR * turn * turn));
        }

        // flip sign of all driving power if we are in REVERSE mode
        if (mcdrive.isDirectionReverse()) {
            power = -power;
            strafe = -strafe;
        }
        if (rrmdrive != null) {
            driveRRM(power, strafe, turn);
        } else {
            driveMecabot(power, strafe, turn);
        }
    }

    public void driveRRM(double power, double strafe, double turn) {

        rrmdrive.setWeightedDrivePower(
                new Pose2d(power, strafe, turn)
        );

        // Update the drive class
        rrmdrive.update();
    }


    public void driveMecabot(double power, double strafe, double turn) {
        // when we want to move sideways (MECANUM)
        if (Math.abs(strafe) > 0) {
            mcdrive.driveMecanum(strafe);
            telemetry.addData("Mecanum ", "%.2f", power);
        }
        // normal tank movement
        else {  // when joystick is inactive, this applies brakes, be careful to avoid during autodrive
            // forward press on joystick is negative, backward press (towards human) is positive
            // right press on joystick is positive value, left press is negative value
            // reverse sign of joystick values to match the expected sign in driveTank() method.

            mcdrive.driveTank(power, turn);
            //telemetry.addData("Tank Power", "Drive=%.2f Turn=%.2f", power, turn);
        }
    }

    public void driveAuto() {

        if (autoDriving) {
            if (rrmdrive.isBusy()) {
                rrmdrive.update();
            }
            else {
                clearAutoDriving();
            }
        }
    }
    protected void setAutoDriving() {
        autoDriving = true;
    }
    protected void clearAutoDriving() {
        autoDriving = false;
    }
}
