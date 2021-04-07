package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Robot Rotation using Gyro reading for current position and simple turn driving overshoots the target position.
 * We use PID controller to rotate to desired heading accurately
 * The P,I,D coefficients need to be determined by tuning, otherwise the robot may not even move.
 *
 */
@TeleOp(name = "Gyro Rotate PID Tuner", group = "Test")
@Disabled                            // Comment this out to add to the opmode list
public class GyroRotatePIDTuner extends LinearOpMode {


    @Override
    public void runOpMode() {

        waitForStart();

        while(!(gamepad1.x) && opModeIsActive()) {
            idle();
        }

        while(opModeIsActive()){
            telemetry.addData("Gyro Rotation ", "Complete");

            //Update values
            telemetry.update();
        }
    }

}
