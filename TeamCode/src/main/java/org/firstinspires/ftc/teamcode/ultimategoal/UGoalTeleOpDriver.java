package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecabotDrive;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TeleOpDriver;

import static org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.*;

/* Button Controls (programmers please keep this list up to date)
 * All buttons coded in this file are on GamePad1 unless specified otherwise
 * START + X = Reset Position, odometry, IMU, Lift intake, wobble arm encoder
 *
 * A = Turn shooter towards Goals
 * B = Turn intake towards Goals
 * X = Reset Odometry at Left Wall
 * Y = Reset Odometry at Right Wall
 *
 * LEFT_JOYSTICK_Y  = Manual Driving Forward
 * RIGHT_JOYSTICK_X = Manual Driving Turn
 * RIGHT_TRIGGER    = Mecanum Strafe Right
 * LEFT_TRIGGER     = Mecanum Strafe Left
 *
 * DPAD_UP          = Auto Drive to High Goal
 * DPAD_DOWN        = Auto Drive to Power Shot 3
 * DPAD_RIGHT       = Rotate Right by 1 power shot
 * DPAD_LEFT        = Rotate Left by 1 power shot
 * LEFT_BUMPER+RIGHT_BUMPER = Auto Drive to Wobble Drop Zone
 *
 * RIGHT_BUMPER     = Drive FORWARD direction
 * LEFT_BUMPER      = Drive REVERSE direction
 * START + RIGHT_BUMPER = Drive MAX speed
 * START + LEFT_BUMPER = Drive DEFAULT speed
 */

public class UGoalTeleOpDriver extends TeleOpDriver {

    UGoalRobot robot;
    boolean toggle;

    /* Constructor */
    public UGoalTeleOpDriver(LinearOpMode opMode, RRMecanumDrive rrmdrive, MecabotDrive mcdrive, UGoalRobot aRobot) {
        super(opMode, rrmdrive, mcdrive);
        robot = aRobot;
    }

    @Override
    public void  driveGameTeleOp() {
        if (gamepad1.a) {      // Turn shooter towards Goals
            mcdrive.getOdometry().setGlobalPosition(poseHighGoalTeleOp.getX(), poseHighGoalTeleOp.getY(), poseHighGoalTeleOp.getHeading());
            rrmdrive.initIMU();
            rrmdrive.setPoseEstimate(poseHighGoalTeleOp);
        }
        else if (gamepad1.b) {      // Turn Intake towards Goals
            mcdrive.gyroRotateToHeading(ANGLE_NEG_X_AXIS);
        }
        else if (gamepad1.x) {      // Reset odometry values aligned against left perimeter wall
            rrmdrive.initIMU();
            rrmdrive.setPoseEstimate(poseOdoLeft);
            mcdrive.getOdometry().setGlobalPosition(poseOdoLeft.getX(), poseOdoLeft.getY(), poseOdoLeft.getHeading());
        }
        else if (gamepad1.y) {      // Reset odometry values aligned against right perimeter wall
            rrmdrive.initIMU();
            rrmdrive.setPoseEstimate(poseOdoRight);
            mcdrive.getOdometry().setGlobalPosition(poseOdoRight.getX(), poseOdoRight.getY(), poseOdoRight.getHeading());
        }
    }

    // disabling this functionality due to lack of buttons
    //mcdrive.gyroRotateToHeading(ANGLE_POS_X_AXIS, MecabotDrive.ROTATE_SPEED_SLOW, MecabotDrive.TIMEOUT_SHORT);

    @Override
    public void  driveGameAuto() {

        // NOTE NOTE: The shooting platform tilt is included at the end of auto driving
        if (gamepad1.dpad_up) {
            setAutoDriving();
            robot.rrdriveToTarget(Target.HIGHGOAL);
        }
        if (gamepad1.dpad_down) {
            setAutoDriving();
            robot.rrdriveToTarget(Target.POWERSHOT_3);
        }
        if (gamepad1.dpad_left) {
            //mcdrive.gyroRotate(ROBOT_ROTATE_POWERSHOT);
            Trajectory goToTarget = rrmdrive.trajectoryBuilder(rrmdrive.getPoseEstimate())
                    .strafeLeft(DISTANCE_BETWEEN_POWERSHOT)
                    .build();
            rrmdrive.followTrajectory(goToTarget);
        }
        if (gamepad1.dpad_right) {
            //mcdrive.gyroRotate(-ROBOT_ROTATE_POWERSHOT);
            Trajectory goToTarget = rrmdrive.trajectoryBuilder(rrmdrive.getPoseEstimate())
                    .strafeRight(DISTANCE_BETWEEN_POWERSHOT)
                    .build();
            rrmdrive.followTrajectory(goToTarget);
        }
        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            toggle = !toggle;
            setAutoDriving();
            if (toggle) {
                robot.rrdriveToDropZone(Target.WOBBLE_LANDING_1);
            } else {
                robot.rrdriveToDropZone(Target.WOBBLE_LANDING_2);
            }
        }
    }

}
