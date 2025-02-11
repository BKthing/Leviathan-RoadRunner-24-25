package org.firstinspires.ftc.teamcode.util.hardwareQueue;

public class DevicePair implements Comparable<DevicePair> {
    private final int id;
    private final double priorityVal;

    public DevicePair(int id, double priorityVal) {
        this.id = id;
        this.priorityVal = priorityVal;
    }

    public int getId() {
        return id;
    }

    public double getPriorityVal() {
        return priorityVal;
    }

    @Override
    public int compareTo(DevicePair devicePair) {
        return Double.compare(devicePair.getPriorityVal(), priorityVal);
    }
}
