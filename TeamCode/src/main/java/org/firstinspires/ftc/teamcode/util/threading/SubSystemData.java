package org.firstinspires.ftc.teamcode.util.threading;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.reefsharklibrary.robotControl.HardwareQueue;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SubSystemData {
    private final HardwareMap hardwareMap;
    private final HardwareQueue hardwareQueue;
    private final Telemetry telemetry;
    private final Gamepad gamepad1Instance;
    private final Gamepad gamepad2Instance;

    public SubSystemData(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Telemetry telemetry, Gamepad gamepad1Instance, Gamepad gamepad2Instance) {
        this.hardwareMap = hardwareMap;
        this.hardwareQueue = hardwareQueue;
        this.telemetry = telemetry;
        this.gamepad1Instance = gamepad1Instance;
        this.gamepad2Instance = gamepad2Instance;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public HardwareQueue getHardwareQueue() {
        return hardwareQueue;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public Gamepad getGamepad1Instance() {
        return gamepad1Instance;
    }

    public Gamepad getGamepad2Instance() {
        return gamepad2Instance;
    }

}
