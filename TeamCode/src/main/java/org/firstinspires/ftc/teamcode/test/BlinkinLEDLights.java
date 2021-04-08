package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Mecabot;

@TeleOp(name = "Blinkin LED Lights", group = "Test")
@Disabled
public class BlinkinLEDLights extends LinearOpMode{
    
    Mecabot mecabot;
    boolean debounce = false;
    public RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;

    @Override public void runOpMode() {
        // Create main OpMode member objects initialize the hardware variables.
        mecabot = new Mecabot(hardwareMap);
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
                mecabot.lights.setPattern(pattern);
                debounce = true;
            }
        } else if (gamepad2.dpad_left){
            if (!debounce) {
                pattern = pattern.previous();
                mecabot.lights.setPattern(pattern);
                debounce = true;
            }
        } else{
            debounce = false;
        }

    }
    public void Telemetry(){
        telemetry.addLine("Light:")
        .addData("Current Pattern", pattern.toString());
        telemetry.update();
    }

}
