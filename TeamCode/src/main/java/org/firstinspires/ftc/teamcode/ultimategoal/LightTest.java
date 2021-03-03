package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.MecabotDrive;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TeleOpDriver;

import static org.firstinspires.ftc.teamcode.ultimategoal.FieldUGoal.*;
import static org.firstinspires.ftc.teamcode.drive.RRMecanumDrive.savedPose;

@TeleOp(name = "Light test")

public class LightTest extends LinearOpMode{
    
    RRMecanumDrive rrmdrive;
    MecabotDrive mcdrive;
    UGoalRobot robot;
    TeleOpDriver driver;
    Telemetry drvrTelemetry;
    Telemetry dashTelemetry;
    boolean debounce = false;
    public RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;

    @Override public void runOpMode() {
        // Redirect telemetry printouts to both Driver Station and FTC Dashboard
        dashTelemetry = FtcDashboard.getInstance().getTelemetry();
        drvrTelemetry = telemetry;
        //telemetry = new MultipleTelemetry(drvrTelemetry, dashTelemetry);
        // Create main OpMode member objects initialize the hardware variables.
        robot = new UGoalRobot(hardwareMap,this);
        driver = new UGoalTeleOpDriver(this, rrmdrive, mcdrive, robot);
        mcdrive = robot.getMCBdrive();
        rrmdrive = robot.getRRMdrive();
        //pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;

        while(opModeIsActive()){
            light();
            Telemetry();
        }
    }
    
    public void light(){
        if (gamepad2.dpad_right) {
            if (!debounce) {
                pattern = pattern.next();
                mcdrive.lights.setPattern(pattern);
                debounce = true;
            }
        } else if (gamepad2.dpad_left){
            if (!debounce) {
                pattern = pattern.previous();
                mcdrive.lights.setPattern(pattern);
                debounce = true;
            }
        } else{
            debounce = false;
        }

    }
    public void Telemetry(){
        drvrTelemetry.addLine("Light:")
        .addData("Current Pattern", pattern.toString());
        drvrTelemetry.update();
    }

}
