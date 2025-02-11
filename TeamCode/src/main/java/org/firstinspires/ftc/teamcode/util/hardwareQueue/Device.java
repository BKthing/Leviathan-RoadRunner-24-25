package org.firstinspires.ftc.teamcode.util.hardwareQueue;

public interface Device extends Comparable<Device> {

//    void priorityUpdate();
    void update();

    void updatePriorityVal();
    double getPriorityVal();

    double getTimeSinceLastUpdate();

    double getEstimatedAMPS();

    String getDeviceName();

    int getId();

    void setId(int id);

    DevicePair getDevicePair();


}
