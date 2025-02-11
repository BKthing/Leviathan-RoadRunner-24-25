package org.firstinspires.ftc.teamcode.util.hardwareQueue;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class VoltageRegulatingHardwareQueue {
    private final List<HardwareGroup> hardwareGroups = new ArrayList<>();

    private final List<Device> devices = new ArrayList<>();

    private Queue<Integer> updateQueue;

    private final VoltageSensor voltageSensor;

    private final double targetOperatingVoltage = 12.0;
    private final double criticalOperatingVoltage = 8.0;

    public VoltageRegulatingHardwareQueue(HardwareMap hardwareMap) {
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void addHardwareGroup(HardwareGroup hardwareGroup) {
        hardwareGroups.add(hardwareGroup);
    }

    public HardwareGroup getHardwareGroup(int hardwareGroup) {
        return hardwareGroups.get(hardwareGroup);
    }

    public void addDevice(Device device) {
        addDevice(device, 1);
    }

    public void addDevice(Device device, int hardwareGroup) {
        device.setId(devices.size());
        devices.add(device);
        hardwareGroups.get(hardwareGroup).addDevice(devices.get(devices.size()-1));
    }

    public Device getDevice(int device) {
        return devices.get(device);
    }

    public ThreadMotor getThreadMotor(int device) {
        if (devices.get(device).getClass() == ThreadMotor.class) {
            return (ThreadMotor) devices.get(device);
        } else {
            throw new RuntimeException("Device is not of type ThreadMotor");
        }
    }

    public void updateQueue(HardwareMap hardwareMap) {

        if (voltageSensor.getVoltage()<criticalOperatingVoltage) {
            //lower device current draw as necessary
        } else if (voltageSensor.getVoltage()<targetOperatingVoltage) {
            //lower device current draw while respecting device limits
        }

        List<DevicePair> devicePairs = new LinkedList<>();

        for (Device device : devices) {
            device.updatePriorityVal();
            devicePairs.add(device.getDevicePair());
        }

        Collections.sort(devicePairs);

        updateQueue.clear();

        for (DevicePair devicePair : devicePairs) {
            updateQueue.add(devicePair.getId());
        }

    }

    public boolean updateSingle() {
        if (updateQueue.size()>0) {
            devices.get(updateQueue.remove()).update();
            return true;
        } else {
            return false;
        }
    }

}
