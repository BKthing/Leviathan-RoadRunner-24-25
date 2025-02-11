package org.firstinspires.ftc.teamcode.util.hardwareQueue;

import java.util.ArrayList;
import java.util.List;

public class HardwareGroup {

    List<Device> devices = new ArrayList<>();

    final int id;
    int voltagePriority = 1;

    double voltageMultiplier = 1;
    double minVoltageMultiplier = .2;

    public HardwareGroup(int id) {
        this.id = id;
    }

    public void setVoltagePriority(int priority) {
        voltagePriority = priority;
    }

    public void addDevice(Device device) {
        devices.add(device);
    }

    public double getEstimatedWatts() {
        double watts = 0;
        for (Device device : devices) {
            watts += device.getEstimatedAMPS();
        }
        return watts;
    }

    public double getVoltageMultiplier() {
        return voltageMultiplier;
    }

    public void setVoltageMultiplier(double voltageMultiplier) {
        this.voltageMultiplier = voltageMultiplier;
    }

    public double getMinVoltageMultiplier() {
        return minVoltageMultiplier;
    }

    public void setMinVoltageMultiplier(double minVoltageMultiplier) {
        this.minVoltageMultiplier = minVoltageMultiplier;
    }

    public int getId() {
        return id;
    }
}
