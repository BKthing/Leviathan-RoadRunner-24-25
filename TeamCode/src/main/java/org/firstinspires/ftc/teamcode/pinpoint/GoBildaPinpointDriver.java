package org.firstinspires.ftc.teamcode.pinpoint;

import androidx.annotation.RequiresApi;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;

@I2cDeviceType
@DeviceProperties(
        name = "goBILDA® Pinpoint Odometry Computer",
        xmlTag = "goBILDAPinpoint",
        description = "goBILDA® Pinpoint Odometry Computer (IMU Sensor Fusion for 2 Wheel Odometry)"
)
public class GoBildaPinpointDriver extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {
    private int deviceStatus = 0;
    private int loopTime = 0;
    private int xEncoderValue = 0;
    private int yEncoderValue = 0;
    private float xPosition = 0.0F;
    private float yPosition = 0.0F;
    private float hOrientation = 0.0F;
    private float xVelocity = 0.0F;
    private float yVelocity = 0.0F;
    private float hVelocity = 0.0F;
    private static final float goBILDA_SWINGARM_POD = 13.262912F;
    private static final float goBILDA_4_BAR_POD = 19.894367F;
    public static final byte DEFAULT_ADDRESS = 49;

    public GoBildaPinpointDriver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(49));
        super.registerArmingStateCallback(false);
    }

    public HardwareDevice.Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    protected synchronized boolean doInitialize() {
        ((LynxI2cDeviceSynch)this.deviceClient).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        return true;
    }

    public String getDeviceName() {
        return "goBILDA® Pinpoint Odometry Computer";
    }

    private void writeInt(Register reg, int i) {
        this.deviceClient.write(reg.bVal, TypeConversion.intToByteArray(i, ByteOrder.LITTLE_ENDIAN));
    }

    private int readInt(Register reg) {
        return TypeConversion.byteArrayToInt(this.deviceClient.read(reg.bVal, 4), ByteOrder.LITTLE_ENDIAN);
    }

    private float byteArrayToFloat(byte[] byteArray, ByteOrder byteOrder) {
        return ByteBuffer.wrap(byteArray).order(byteOrder).getFloat();
    }

    private float readFloat(Register reg) {
        return this.byteArrayToFloat(this.deviceClient.read(reg.bVal, 4), ByteOrder.LITTLE_ENDIAN);
    }

    private byte[] floatToByteArray(float value, ByteOrder byteOrder) {
        return ByteBuffer.allocate(4).order(byteOrder).putFloat(value).array();
    }

    private void writeByteArray(Register reg, byte[] bytes) {
        this.deviceClient.write(reg.bVal, bytes);
    }

    private void writeFloat(Register reg, float f) {
        byte[] bytes = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putFloat(f).array();
        this.deviceClient.write(reg.bVal, bytes);
    }

    private DeviceStatus lookupStatus(int s) {
        if ((s & GoBildaPinpointDriver.DeviceStatus.CALIBRATING.status) != 0) {
            return GoBildaPinpointDriver.DeviceStatus.CALIBRATING;
        } else {
            boolean xPodDetected = (s & GoBildaPinpointDriver.DeviceStatus.FAULT_X_POD_NOT_DETECTED.status) == 0;
            boolean yPodDetected = (s & GoBildaPinpointDriver.DeviceStatus.FAULT_Y_POD_NOT_DETECTED.status) == 0;
            if (!xPodDetected && !yPodDetected) {
                return GoBildaPinpointDriver.DeviceStatus.FAULT_NO_PODS_DETECTED;
            } else if (!xPodDetected) {
                return GoBildaPinpointDriver.DeviceStatus.FAULT_X_POD_NOT_DETECTED;
            } else if (!yPodDetected) {
                return GoBildaPinpointDriver.DeviceStatus.FAULT_Y_POD_NOT_DETECTED;
            } else if ((s & GoBildaPinpointDriver.DeviceStatus.FAULT_IMU_RUNAWAY.status) != 0) {
                return GoBildaPinpointDriver.DeviceStatus.FAULT_IMU_RUNAWAY;
            } else {
                return (s & GoBildaPinpointDriver.DeviceStatus.READY.status) != 0 ? GoBildaPinpointDriver.DeviceStatus.READY : GoBildaPinpointDriver.DeviceStatus.NOT_READY;
            }
        }
    }

    @RequiresApi(
            api = 9
    )
    public void update() {
        byte[] bArr = this.deviceClient.read(GoBildaPinpointDriver.Register.BULK_READ.bVal, 40);
        this.deviceStatus = TypeConversion.byteArrayToInt(Arrays.copyOfRange(bArr, 0, 4), ByteOrder.LITTLE_ENDIAN);
        this.loopTime = TypeConversion.byteArrayToInt(Arrays.copyOfRange(bArr, 4, 8), ByteOrder.LITTLE_ENDIAN);
        this.xEncoderValue = TypeConversion.byteArrayToInt(Arrays.copyOfRange(bArr, 8, 12), ByteOrder.LITTLE_ENDIAN);
        this.yEncoderValue = TypeConversion.byteArrayToInt(Arrays.copyOfRange(bArr, 12, 16), ByteOrder.LITTLE_ENDIAN);
        this.xPosition = this.byteArrayToFloat(Arrays.copyOfRange(bArr, 16, 20), ByteOrder.LITTLE_ENDIAN);
        this.yPosition = this.byteArrayToFloat(Arrays.copyOfRange(bArr, 20, 24), ByteOrder.LITTLE_ENDIAN);
        this.hOrientation = this.byteArrayToFloat(Arrays.copyOfRange(bArr, 24, 28), ByteOrder.LITTLE_ENDIAN);
        this.xVelocity = this.byteArrayToFloat(Arrays.copyOfRange(bArr, 28, 32), ByteOrder.LITTLE_ENDIAN);
        this.yVelocity = this.byteArrayToFloat(Arrays.copyOfRange(bArr, 32, 36), ByteOrder.LITTLE_ENDIAN);
        this.hVelocity = this.byteArrayToFloat(Arrays.copyOfRange(bArr, 36, 40), ByteOrder.LITTLE_ENDIAN);
    }

    public void update(readData data) {
        if (data == GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING) {
            this.hOrientation = this.byteArrayToFloat(this.deviceClient.read(GoBildaPinpointDriver.Register.H_ORIENTATION.bVal, 4), ByteOrder.LITTLE_ENDIAN);
        }

    }

    public void setOffsets(double xOffset, double yOffset) {
        this.writeFloat(GoBildaPinpointDriver.Register.X_POD_OFFSET, (float)xOffset);
        this.writeFloat(GoBildaPinpointDriver.Register.Y_POD_OFFSET, (float)yOffset);
    }

    public void recalibrateIMU() {
        this.writeInt(GoBildaPinpointDriver.Register.DEVICE_CONTROL, 1);
    }

    public void resetPosAndIMU() {
        this.writeInt(GoBildaPinpointDriver.Register.DEVICE_CONTROL, 2);
    }

    public void setEncoderDirections(EncoderDirection xEncoder, EncoderDirection yEncoder) {
        if (xEncoder == GoBildaPinpointDriver.EncoderDirection.FORWARD) {
            this.writeInt(GoBildaPinpointDriver.Register.DEVICE_CONTROL, 32);
        }

        if (xEncoder == GoBildaPinpointDriver.EncoderDirection.REVERSED) {
            this.writeInt(GoBildaPinpointDriver.Register.DEVICE_CONTROL, 16);
        }

        if (yEncoder == GoBildaPinpointDriver.EncoderDirection.FORWARD) {
            this.writeInt(GoBildaPinpointDriver.Register.DEVICE_CONTROL, 8);
        }

        if (yEncoder == GoBildaPinpointDriver.EncoderDirection.REVERSED) {
            this.writeInt(GoBildaPinpointDriver.Register.DEVICE_CONTROL, 4);
        }

    }

    public void setEncoderResolution(GoBildaOdometryPods pods) {
        if (pods == GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD) {
            this.writeByteArray(GoBildaPinpointDriver.Register.MM_PER_TICK, this.floatToByteArray(13.262912F, ByteOrder.LITTLE_ENDIAN));
        }

        if (pods == GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD) {
            this.writeByteArray(GoBildaPinpointDriver.Register.MM_PER_TICK, this.floatToByteArray(19.894367F, ByteOrder.LITTLE_ENDIAN));
        }

    }

    public void setEncoderResolution(double ticks_per_mm) {
        this.writeByteArray(GoBildaPinpointDriver.Register.MM_PER_TICK, this.floatToByteArray((float)ticks_per_mm, ByteOrder.LITTLE_ENDIAN));
    }

    public void setYawScalar(double yawOffset) {
        this.writeByteArray(GoBildaPinpointDriver.Register.YAW_SCALAR, this.floatToByteArray((float)yawOffset, ByteOrder.LITTLE_ENDIAN));
    }

    public Pose2D setPosition(Pose2D pos) {
        this.writeByteArray(GoBildaPinpointDriver.Register.X_POSITION, this.floatToByteArray((float)pos.getX(DistanceUnit.MM), ByteOrder.LITTLE_ENDIAN));
        this.writeByteArray(GoBildaPinpointDriver.Register.Y_POSITION, this.floatToByteArray((float)pos.getY(DistanceUnit.MM), ByteOrder.LITTLE_ENDIAN));
        this.writeByteArray(GoBildaPinpointDriver.Register.H_ORIENTATION, this.floatToByteArray((float)pos.getHeading(AngleUnit.RADIANS), ByteOrder.LITTLE_ENDIAN));
        return pos;
    }

    public int getDeviceID() {
        return this.readInt(GoBildaPinpointDriver.Register.DEVICE_ID);
    }

    public int getDeviceVersion() {
        return this.readInt(GoBildaPinpointDriver.Register.DEVICE_VERSION);
    }

    public float getYawScalar() {
        return this.readFloat(GoBildaPinpointDriver.Register.YAW_SCALAR);
    }

    public DeviceStatus getDeviceStatus() {
        return this.lookupStatus(this.deviceStatus);
    }

    public int getLoopTime() {
        return this.loopTime;
    }

    public double getFrequency() {
        return this.loopTime != 0 ? 1000000.0 / (double)this.loopTime : 0.0;
    }

    public int getEncoderX() {
        return this.xEncoderValue;
    }

    public int getEncoderY() {
        return this.yEncoderValue;
    }

    public double getPosX() {
        return (double)this.xPosition;
    }

    public double getPosY() {
        return (double)this.yPosition;
    }

    public double getHeading() {
        return (double)this.hOrientation;
    }

    public double getVelX() {
        return (double)this.xVelocity;
    }

    public double getVelY() {
        return (double)this.yVelocity;
    }

    public double getHeadingVelocity() {
        return (double)this.hVelocity;
    }

    public float getXOffset() {
        return this.readFloat(GoBildaPinpointDriver.Register.X_POD_OFFSET);
    }

    public float getYOffset() {
        return this.readFloat(GoBildaPinpointDriver.Register.Y_POD_OFFSET);
    }

    public Pose2D getPosition() {
        return new Pose2D(DistanceUnit.MM, (double)this.xPosition, (double)this.yPosition, AngleUnit.RADIANS, (double)this.hOrientation);
    }

    public Pose2D getVelocity() {
        return new Pose2D(DistanceUnit.MM, (double)this.xVelocity, (double)this.yVelocity, AngleUnit.RADIANS, (double)this.hVelocity);
    }

    private static enum Register {
        DEVICE_ID(1),
        DEVICE_VERSION(2),
        DEVICE_STATUS(3),
        DEVICE_CONTROL(4),
        LOOP_TIME(5),
        X_ENCODER_VALUE(6),
        Y_ENCODER_VALUE(7),
        X_POSITION(8),
        Y_POSITION(9),
        H_ORIENTATION(10),
        X_VELOCITY(11),
        Y_VELOCITY(12),
        H_VELOCITY(13),
        MM_PER_TICK(14),
        X_POD_OFFSET(15),
        Y_POD_OFFSET(16),
        YAW_SCALAR(17),
        BULK_READ(18);

        private final int bVal;

        private Register(int bVal) {
            this.bVal = bVal;
        }
    }

    public static enum DeviceStatus {
        NOT_READY(0),
        READY(1),
        CALIBRATING(2),
        FAULT_X_POD_NOT_DETECTED(4),
        FAULT_Y_POD_NOT_DETECTED(8),
        FAULT_NO_PODS_DETECTED(12),
        FAULT_IMU_RUNAWAY(16);

        private final int status;

        private DeviceStatus(int status) {
            this.status = status;
        }
    }

    public static enum readData {
        ONLY_UPDATE_HEADING;

        private readData() {
        }
    }

    public static enum EncoderDirection {
        FORWARD,
        REVERSED;

        private EncoderDirection() {
        }
    }

    public static enum GoBildaOdometryPods {
        goBILDA_SWINGARM_POD,
        goBILDA_4_BAR_POD;

        private GoBildaOdometryPods() {
        }
    }
}
