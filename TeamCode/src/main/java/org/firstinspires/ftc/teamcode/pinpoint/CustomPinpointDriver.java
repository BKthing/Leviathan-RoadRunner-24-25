package org.firstinspires.ftc.teamcode.pinpoint;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.reefsharklibrary.data.Rotation;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.Objects;

public class CustomPinpointDriver {
    private HardwareMap hardwareMap;
    private GoBildaPinpointDriver odo;
    private double previousHeading;
    private double totalHeading;
    private Pose2d startPose;
    private long deltaTimeNano;
    private ElapsedTimer timer;
    private PoseVelocity2d currentVelocity;
    private Pose2d pinpointPose;
    private boolean pinpointCooked;

    public CustomPinpointDriver(HardwareMap map, PinpointDrive.Params params) {
        this(map, new Pose2d(0, 0, 0), params);
    }

    public CustomPinpointDriver(HardwareMap map, Pose2d setStartPose, PinpointDrive.Params params) {
        this.pinpointCooked = false;
        this.hardwareMap = map;
        this.odo = (GoBildaPinpointDriver)this.hardwareMap.get(GoBildaPinpointDriver.class, params.pinpointDeviceName);
        this.setOffsets(DistanceUnit.MM.fromInches(params.xOffset), DistanceUnit.MM.fromInches(params.yOffset));
//        if (PinpointConstants.useYawScalar) {
//            this.odo.setYawScalar(PinpointConstants.yawScalar);
//        }

//        if (PinpointConstants.useCustomEncoderResolution) {
//            this.odo.setEncoderResolution(PinpointConstants.customEncoderResolution);
//        } else {
        this.odo.setEncoderResolution(params.encoderResolution);
//        }

        this.odo.setEncoderDirections(params.xDirection, params.yDirection);
        this.resetPinpoint();
        this.setStartPose(setStartPose);
        this.totalHeading = 0.0;
        this.timer = new ElapsedTimer();
        this.pinpointPose = this.startPose;
        this.currentVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
        this.deltaTimeNano = 1L;
        this.previousHeading = setStartPose.heading.toDouble();
    }

    public Pose2d getPose() {
        return this.pinpointPose;
    }

    public PoseVelocity2d getVelocity() {
        return this.currentVelocity;
    }

    public Vector2d getVelocityVector() {
        return this.currentVelocity.linearVel;
    }

    public void setStartPose(Pose2d setStart) {
        if (!Objects.equals(this.startPose, new Pose2d(0, 0, 0)) && this.startPose != null) {
            Pose2d currentPose = MathUtil.toRoadRunnerPose(MathUtil.toReefSharkPose(this.pinpointPose).rotateVector(-this.startPose.heading.toDouble()).minus(MathUtil.toReefSharkPose(this.startPose)));//MathFunctions.subtractPoses(MathFunctions.rotatePose(this.pinpointPose, -this.startPose.getHeading(), false), this.startPose);
            this.setPose(MathUtil.toRoadRunnerPose(MathUtil.toReefSharkPose(setStart).plus(MathUtil.toReefSharkPose(currentPose).rotateVector(setStart.heading.toDouble()))));//MathFunctions.addPoses(setStart, MathFunctions.rotatePose(currentPose, setStart.getHeading(), false))
        } else {
            this.setPose(setStart);
        }

        this.startPose = setStart;
    }

    public void setPose(Pose2d setPose) {
        this.odo.setPosition(new Pose2D(DistanceUnit.INCH, setPose.position.x, setPose.position.y, AngleUnit.RADIANS, setPose.heading.toDouble()));
        this.pinpointPose = setPose;
        this.previousHeading = setPose.heading.toDouble();
    }

    public void update() {
        this.deltaTimeNano = (long) this.timer.nanoSeconds();
        this.timer.reset();
        this.odo.update();
        Pose2d currentPinpointPose = this.getPoseEstimate(this.odo.getPosition(), this.pinpointPose, this.deltaTimeNano);
        this.totalHeading += Rotation.inRange(currentPinpointPose.heading.toDouble()-this.previousHeading, Math.PI, -Math.PI);//MathFunctions.getSmallestAngleDifference(currentPinpointPose.getHeading(), this.previousHeading);
        this.previousHeading = currentPinpointPose.heading.toDouble();
        Pose2d deltaPose = MathUtil.toRoadRunnerPose(MathUtil.toReefSharkPose(currentPinpointPose).minus(MathUtil.toReefSharkPose(this.pinpointPose)));// MathFunctions.subtractPoses(currentPinpointPose, this.pinpointPose);
        this.currentVelocity = new PoseVelocity2d(new Vector2d(deltaPose.position.x / ((double) this.deltaTimeNano / Math.pow(10.0, 9.0)), deltaPose.position.y / ((double) this.deltaTimeNano / Math.pow(10.0, 9.0))), deltaPose.heading.toDouble() / ((double) this.deltaTimeNano / Math.pow(10.0, 9.0)));// new Pose(deltaPose.getX() / ((double)this.deltaTimeNano / Math.pow(10.0, 9.0)), deltaPose.getY() / ((double)this.deltaTimeNano / Math.pow(10.0, 9.0)), deltaPose.getHeading() / ((double)this.deltaTimeNano / Math.pow(10.0, 9.0)));
        this.pinpointPose = currentPinpointPose;
    }

    public double getTotalHeading() {
        return this.totalHeading;
    }

    public double getForwardMultiplier() {
        return (double)this.odo.getEncoderY();
    }

    public double getLateralMultiplier() {
        return (double)this.odo.getEncoderX();
    }

    public double getTurningMultiplier() {
        return (double)this.odo.getYawScalar();
    }

    private void setOffsets(double xOffset, double yOffset) {
        this.odo.setOffsets(xOffset, yOffset);
    }

    public void resetIMU() throws InterruptedException {
        this.odo.recalibrateIMU();
    }

    private void resetPinpoint() {
        this.odo.resetPosAndIMU();

        try {
            Thread.sleep(300L);
        } catch (InterruptedException var2) {
            InterruptedException e = var2;
            throw new RuntimeException(e);
        }
    }

    private Pose2d getPoseEstimate(Pose2D pinpointEstimate, Pose2d currentPose, long deltaTime) {
        if (!Double.isNaN(pinpointEstimate.getX(DistanceUnit.INCH)) && !Double.isNaN(pinpointEstimate.getY(DistanceUnit.INCH)) && !Double.isNaN(pinpointEstimate.getHeading(AngleUnit.RADIANS))) {
            Pose2d estimate = new Pose2d(pinpointEstimate.getX(DistanceUnit.INCH), pinpointEstimate.getY(DistanceUnit.INCH), pinpointEstimate.getHeading(AngleUnit.RADIANS));
            pinpointCooked = false;
            return estimate;
            //            if (roughlyEquals(estimate, new Pose2d(0, 0, 0), 0.002)) {
//                this.pinpointCooked = true;
//                return MathUtil.toRoadRunnerPose(MathUtil.toReefSharkPose(currentPose).plus(MathUtil.toReefSharkPose(this.currentVelocity).scale((double)deltaTime / Math.pow(10.0, 9.0))));
//            } else {
//                this.pinpointCooked = false;
//                return estimate;
//            }
        } else {
            this.pinpointCooked = true;
            return MathUtil.toRoadRunnerPose(MathUtil.toReefSharkPose(currentPose).plus(MathUtil.toReefSharkPose(this.currentVelocity).scale((double)deltaTime / Math.pow(10.0, 9.0))));
        }
    }

    public boolean isPinpointCooked() {
        Pose2d prevPoseEstimate = pinpointPose;
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.RADIANS, 1));
        Pose2D newPose =  odo.getPosition();
        if (!Double.isNaN(newPose.getX(DistanceUnit.INCH)) && !Double.isNaN(newPose.getY(DistanceUnit.INCH)) && !Double.isNaN(newPose.getHeading(AngleUnit.RADIANS)) && new com.reefsharklibrary.data.Pose2d(newPose.getX(DistanceUnit.INCH), newPose.getY(DistanceUnit.INCH), newPose.getHeading(AngleUnit.RADIANS)).minus(MathUtil.toReefSharkPose(prevPoseEstimate)).minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(.1, .1, .1))) {
            setPose(prevPoseEstimate);
            return false;
        } else {
            return true;
        }
    }

    public boolean fastIsPinpointCooked() {
        return this.pinpointCooked;
    }

    public boolean roughlyEquals(Pose2d pose1, Pose2d pose2, double accuracy) {
        return roughlyEquals(pose1.position.x, pose2.position.x, accuracy) && roughlyEquals(pose1.position.y, pose2.position.y, accuracy) && roughlyEquals(Rotation.inRange(pose1.position.x - pose2.position.x, Math.PI, -Math.PI), 0.0, accuracy);
    }

    public static boolean roughlyEquals(double one, double two, double accuracy) {
        return one < two + accuracy && one > two - accuracy;
    }
}
