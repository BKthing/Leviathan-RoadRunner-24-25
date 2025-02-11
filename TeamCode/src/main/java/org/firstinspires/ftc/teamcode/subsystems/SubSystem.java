package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.reefsharklibrary.robotControl.HardwareQueue;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.ExecutorService;

import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

public abstract class SubSystem {
    public final SubSystemData data;
    public HardwareMap hardwareMap;
    public final Telemetry telemetry;
    public final Gamepad gamepad1, gamepad2;

    public HardwareQueue hardwareQueue;

    Thread updateThread = new Thread();

    public SubSystem(SubSystemData data) {
        this.data = data;
        this.hardwareMap = data.getHardwareMap();
        this.telemetry = data.getTelemetry();
        this.hardwareQueue = data.getHardwareQueue();

        this.gamepad1 = data.getGamepad1Instance();
        this.gamepad2 = data.getGamepad2Instance();
    }

    public void update(ExecutorService es) {
        if (updateThread.isAlive()) throw new RuntimeException("Thread updated while previous thread is still alive");

        priorityData();

        updateThread = new Thread(this::loop);
        es.execute(updateThread);
    }

    //intended to be used for bulk read data and data you can quickly get a response from
    //processing in this should be minimized because it is not threaded
    public abstract void priorityData();

    //used for processing, any hardware calls should either be put in the hardwareQueue or made in priority data
    public abstract void loop();

    public TelemetryPacket dashboard(TelemetryPacket packet) {
        return packet;
    }

    public boolean isFinished() {
        return updateThread.isAlive();
    }


}


