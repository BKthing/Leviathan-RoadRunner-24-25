package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AndySub;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

@TeleOp
public class Andy extends LinearOpMode {

    MasterThread masterThread;
    AndySub andySub;
    @Override
    public void runOpMode() throws InterruptedException {
        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);
        andySub = new AndySub(masterThread.getData());

        masterThread.addSubSystems(andySub);

        waitForStart();
        masterThread.clearBulkCache();

        while (!isStopRequested()) {
            masterThread.unThreadedUpdate();
        }

    }
}
