package org.firstinspires.ftc.teamcode.util.hardwareQueue;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ThreadMotor implements Device {

    private final String name;
    private int id;

//    private final HardwareMap hardwareMap;
    private final DcMotorEx motor;

    private double targetPower = 0;
    private double currentPower = 0;

    private double priorityVal = 0;

//    private double amps = 0;

    private final ElapsedTimer timer = new ElapsedTimer();

    public ThreadMotor(String name, HardwareMap hardwareMap) {
        this.name = name;
//        this.hardwareMap = hardwareMap;
        motor = hardwareMap.get(DcMotorEx.class, name);
    }

//    @Override
//    public void priorityUpdate() {
//        amps = motor.getCurrent(CurrentUnit.AMPS);
//    }

    @Override
    public void update() {
        motor.setPower(targetPower);
        currentPower = targetPower;
    }

    @Override
    public void updatePriorityVal() {
        priorityVal = Math.abs(currentPower-targetPower)*timer.milliSeconds();
    }

    @Override
    public double getPriorityVal() {
        return priorityVal;
    }

    public void setTargetPower(double targetPower) {
        if (this.targetPower == currentPower && targetPower != this.targetPower) {
            timer.reset();
        }
        this.targetPower = targetPower;
    }

    public double getTargetPower() {
        return targetPower;
    }

    public double getCurrentPower() {
        return currentPower;
    }

    @Override
    public double getTimeSinceLastUpdate() {
        return 0;
    }

    @Override
    public double getEstimatedAMPS() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    @Override
    public String getDeviceName() {
        return name;
    }

    @Override
    public int getId() {
        return id;
    }

    @Override
    public void setId(int id) {
        this.id = id;
    }

    @Override
    public DevicePair getDevicePair() {
        return new DevicePair(id, priorityVal);
    }

    public DcMotorEx getMotor() {
        return motor;
    }

    @Override
    public int compareTo(Device device) {
        return Double.compare(device.getPriorityVal(), priorityVal);
    }

//    @Override
//    public double[] getDevicePair() {
//        return new double[] {id, priorityVal};
//    }

}
