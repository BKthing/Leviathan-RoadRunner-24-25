package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PassData;

@TeleOp
public class DisableColorSensor extends LinearOpMode {
    private static Telemetry.Item checkColorSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        checkColorSensor = telemetry.addData("Check Color Sensor", "");
        while (!isStopRequested()) {

            if (gamepad1.dpad_left) {
                PassData.checkingColorSensor = false;
            }
            else if (gamepad1.dpad_right){
                PassData.checkingColorSensor = true;
            }
            checkColorSensor.setValue(PassData.checkingColorSensor);

            telemetry.update();
        }
    }
}
