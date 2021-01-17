package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
            mcdrive.odometryRotateToHeading(ANGLE_POS_X_AXIS, MecabotDrive.ROTATE_SPEED_DEFAULT, MecabotDrive.TIMEOUT_ROTATE);
        }
        else if (gamepad1.b) {
            mcdrive.gyroRotateToHeading(ANGLE_POS_X_AXIS, MecabotDrive.ROTATE_SPEED_DEFAULT, MecabotDrive.TIMEOUT_ROTATE);
        }
    }

    @Override
    public void  driveGameAuto() {

        if (gamepad1.dpad_up) {
            robot.rrdriveToShootTarget(Target.HIGHGOAL);
            setAutoDriving();
        }
        if (gamepad1.dpad_left) {
            robot.rrdriveToShootTarget(Target.POWERSHOT_1);
            setAutoDriving();
        }
        if (gamepad1.dpad_down) {
            robot.rrdriveToShootTarget(Target.POWERSHOT_2);
            setAutoDriving();
        }
        if (gamepad1.dpad_right) {
            robot.rrdriveToShootTarget(Target.POWERSHOT_3);
            setAutoDriving();
        }
    }

}
