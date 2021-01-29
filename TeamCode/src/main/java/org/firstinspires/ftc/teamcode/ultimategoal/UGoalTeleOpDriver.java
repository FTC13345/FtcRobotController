package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecabotDrive;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TeleOpDriver;

import static org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.*;

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
        if (gamepad1.x) {      // Reset odometry values aligned against left perimeter wall
            rrmdrive.setPoseEstimate(poseOdoLeft);
            mcdrive.getOdometry().setGlobalPosition(poseOdoLeft.getX(), poseOdoLeft.getY(), poseOdoLeft.getHeading());
        }
        else if (gamepad1.b) {      // Reset odometry values aligned against right perimeter wall
            rrmdrive.setPoseEstimate(poseOdoRight);
            mcdrive.getOdometry().setGlobalPosition(poseOdoRight.getX(), poseOdoRight.getY(), poseOdoRight.getHeading());
        }
        else if (gamepad1.a) {      // Reset odometry values at the high goal shooting position
            rrmdrive.setPoseEstimate(poseHighGoal);
            mcdrive.getOdometry().setGlobalPosition(poseHighGoal.getX(), poseHighGoal.getY(), poseHighGoal.getHeading());
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
            mcdrive.gyroRotate(ROBOT_ROTATE_POWERSHOT);
        }
        if (gamepad1.dpad_right) {
            mcdrive.gyroRotate(-ROBOT_ROTATE_POWERSHOT);
        }
        if (gamepad1.y) {
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
