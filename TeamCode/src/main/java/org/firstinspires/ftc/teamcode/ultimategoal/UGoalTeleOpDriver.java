package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecabotDrive;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TeleOpDriver;

import static org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.*;

public class UGoalTeleOpDriver extends TeleOpDriver {

    UGoalRobot robot;

    /* Constructor */
    public UGoalTeleOpDriver(LinearOpMode opMode, RRMecanumDrive rrmdrive, MecabotDrive mcdrive, UGoalRobot aRobot) {
        super(opMode, rrmdrive, mcdrive);
        robot = aRobot;
    }

    @Override
    public void  driveGameTeleOp() {
        if (gamepad1.a) {
            // MecabotDrive rotation replaced by RoadRunner rotation. OdometryGlobalPositio is not working reliably
            // mcdrive.odometryRotateToHeading(ANGLE_POS_X_AXIS, MecabotDrive.ROTATE_SPEED_DEFAULT, MecabotDrive.TIMEOUT_ROTATE);
            rrmdrive.turn(Angle.normDelta(ANGLE_POS_X_AXIS - rrmdrive.getPoseEstimate().getHeading()));
        }
        else if (gamepad1.b) {
            mcdrive.gyroRotateToHeading(ANGLE_POS_X_AXIS, MecabotDrive.ROTATE_SPEED_SLOW, MecabotDrive.TIMEOUT_SHORT);
        }
    }

    @Override
    public void  driveGameAuto() {

        // NOTE NOTE: The shooting platform tilt is included at the end of auto driving
        if (gamepad1.dpad_up) {
            setAutoDriving();
            robot.rrdriveToShootTarget(Target.HIGHGOAL);
        }
        if (gamepad1.dpad_down) {
            setAutoDriving();
            robot.rrdriveToShootTarget(Target.POWERSHOT_3);
        }
        if (gamepad1.dpad_left) {
            setAutoDriving();
            rrmdrive.turn(ROBOT_TURN_SMALL);    // left turn is positive angle delta
        }
        if (gamepad1.dpad_right) {
            setAutoDriving();
            rrmdrive.turn(-ROBOT_TURN_SMALL);    // right turn is negative angle delta
        }
    }

}
