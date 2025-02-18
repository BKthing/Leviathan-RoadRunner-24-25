package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.MathUtil.robotToIntakePos;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.reefsharklibrary.data.MotorPowers;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;
import com.reefsharklibrary.misc.ElapsedTimer;
import com.reefsharklibrary.pathing.PIDPointController;
import com.reefsharklibrary.robotControl.ReusableHardwareAction;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

public class NewDrivetrain extends SubSystem {

    public enum DriveState {
        FOLLOW_PATH,
        DRIVER_CONTROL
    }

    private DriveState driveState;

    public final PinpointDrive drive;

    private Action path;

    private boolean followPath = false;

//    private FtcDashboard dashboard;

    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;

    private List<DcMotorEx> drivetrainMotors;

    private final List<ReusableHardwareAction> motorActions;

    private List<Double> actualPowers = Arrays.asList(0.0, 0.0, 0.0, 0.0);

    private final Telemetry.Item motorPowerTelemetry;

    private final Telemetry.Item followState;

//    private final Telemetry.Item targetMotionState;
//
//    private final Telemetry.Item forwardComponentTelemetry;
//
//    private final Telemetry.Item radiansPerInch;

    private TelemetryPacket packet = new TelemetryPacket();

    private Canvas canvas = new Canvas();


    //how much a motor power must change to warrant an update
    private final double minPowerChange = .03;


    private final ReusableHardwareAction voltageSensorHardwareAction;
    private final VoltageSensor batteryVoltageSensor;
    private double voltage = 13, updatedVoltage = 13;

//    private final TrajectorySequenceRunner runner;

    private Pose2d roadRunnerPoseEstimate = new Pose2d(0, 0, 0);
    private Pose2d roadRunnerPoseVelocity = new Pose2d(0, 0, 0);
//    private Pose2d poseAcceleration;

    private boolean fieldCentric = false;
    private double headingOffset = 0;

    private final Telemetry.Item roadRunnerPos;
    private final Telemetry.Item roadRunnerVel;

    private final Telemetry.Item pinPointCookedTelem;

    private final NewIntake intake;
//    private final Telemetry.Item driveTrainLoopTime;

//    private final ElapsedTimer driveTrainLoopTimer = new ElapsedTimer();

    private final ElapsedTimer voltageUpdateTimer = new ElapsedTimer();

    double intakeDistance = 0;

    double intakeY = 0;

    private PIDPointController pidPointController;

    private Pose2d targetHoldPoint = new Pose2d(0, 0, 0);
    private boolean holdPoint = false;

    private boolean pinpointWasCooked = false;

    public AtomicInteger lineNumber = new AtomicInteger();

    public NewDrivetrain(SubSystemData data, NewIntake intake) {
        this(data, DriveState.FOLLOW_PATH, intake);
    }

    public NewDrivetrain(SubSystemData data, DriveState driveState, NewIntake intake) {
        super(data);

        this.driveState = driveState;

        this.intake = intake;


        this.motorActions = Arrays.asList(new ReusableHardwareAction(hardwareQueue), new ReusableHardwareAction(hardwareQueue), new ReusableHardwareAction(hardwareQueue), new ReusableHardwareAction(hardwareQueue));

        this.voltageSensorHardwareAction = new ReusableHardwareAction(hardwareQueue);

        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");//ex 0
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");//
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");//ex 1

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drivetrainMotors = Arrays.asList(frontLeft, backLeft, backRight, frontRight);

        this.drive = new PinpointDrive(hardwareMap, new com.acmerobotics.roadrunner.Pose2d(0, 0, 0));

        pidPointController = new PIDPointController(RobotConstants.pointPID, RobotConstants.headingPID, RobotConstants.trackWidth, RobotConstants.lateralF, RobotConstants.headingF);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        roadRunnerPos = telemetry.addData("Roadrunner pos", "");

        roadRunnerVel = telemetry.addData("Roadrunner vel", "");

        motorPowerTelemetry = telemetry.addData("Motor powers", "");

        followState = telemetry.addData("Follow state", "");

        pinPointCookedTelem = telemetry.addData("Pinpoint cooked", this.drive.pinpoint.isPinpointCooked());
    }

    @Override
    public void priorityData() {
        voltage = updatedVoltage;

    }

    @Override
    public void loop() {

        drive.setVoltage(voltage); lineNumber.set(178);

        //intake servo rotates by 92.804273
        //intake bar rotates by 124
        //300 deg from 0 to 1
        //servo deg to intake deg 1.336145373
        //servo.setPos val to intake deg 400.8436119
        //-3.14961*Math.cos((1-intake.getActualIntakePos())*400.8436119)

        intakeY = intake.getIntakeVerticalOffset(); lineNumber.set(187);

        intakeDistance = intake.getActualSlidePos()+9.59029 + intake.getIntakeHorizontalOffset(); lineNumber.set(189);

//        intakeDistance = 9.59029;


        if (voltageUpdateTimer.milliSeconds()>200) {
            voltageSensorHardwareAction.setAndQueueIfEmpty(() -> {
                updatedVoltage = batteryVoltageSensor.getVoltage();
            });
            voltageUpdateTimer.reset();
        } lineNumber.set(205);

        switch (driveState) {
            case FOLLOW_PATH:
                lineNumber.set(210);

                if (followPath) {
                    lineNumber.set(213);
                    packet = new TelemetryPacket();
                    packet.fieldOverlay().getOperations().addAll(canvas.getOperations());

                    followPath = path.run(packet);
                    lineNumber.set(217);

                    if (!holdPoint) {
                        lineNumber.set(220);
                        setDrivePower(drive.getDrivePowers());
                    } else {
                        lineNumber.set(223);
                        drive.updatePoseEstimate();
                    }

                    followState.setValue("FOLLOW");
                } else {
                    lineNumber.set(229);
                    drive.updatePoseEstimate();
                }

                roadRunnerPoseEstimate = new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble());  lineNumber.set(193);
                roadRunnerPoseVelocity = new Pose2d(drive.getVelocity().linearVel.x, drive.getVelocity().linearVel.y, drive.getVelocity().angVel);

                lineNumber.set(236);
                if (holdPoint) {
                    MotorPowers motorPowers = new MotorPowers();
                    pidPointController.calculatePowers(roadRunnerPoseEstimate, roadRunnerPoseVelocity, targetHoldPoint, motorPowers);
                    setDrivePower(motorPowers.getNormalizedVoltages(voltage));
                }

                if (this.drive.pinpoint.isPinpointCooked() && pinpointWasCooked) {
                    throw new RuntimeException("Pinpoint cooked in auto");
                }

                pinpointWasCooked = this.drive.pinpoint.isPinpointCooked();
                pinPointCookedTelem.setValue(pinpointWasCooked);

                lineNumber.set(243);
                break;
            case DRIVER_CONTROL:
                lineNumber.set(246);
                followState.setValue("DRIVER");

//                drive.updatePoseEstimate();

                lineNumber.set(251);
                roadRunnerPoseEstimate = new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble());  lineNumber.set(193);
                roadRunnerPoseVelocity = new Pose2d(drive.getVelocity().linearVel.x, drive.getVelocity().linearVel.y, drive.getVelocity().angVel);


                double relativeHeading = roadRunnerPoseEstimate.getHeading()-headingOffset;

                double speedMultiplier = 1- gamepad1.right_trigger*.7;

                MotorPowers powers = new MotorPowers();

                double forward;
                if (Math.abs(gamepad1.left_stick_y)>.02) {
                    forward = -gamepad1.left_stick_y*Math.abs(gamepad1.left_stick_y)*speedMultiplier;
                } else {
                    forward = 0;
                }

                double strafing;
                if (Math.abs(gamepad1.left_stick_x)>.02) {
                    strafing = -gamepad1.left_stick_x*Math.abs(gamepad1.left_stick_x)*speedMultiplier;
                } else {
                    strafing = 0;
                }

                double turn;
                if (Math.abs(gamepad1.right_stick_x)>.02) {
                    turn = -gamepad1.right_stick_x*gamepad1.right_stick_x*gamepad1.right_stick_x*.4*speedMultiplier; //(gamepad1.right_stick_x*Math.abs(gamepad1.right_stick_x))*.7;
                } else {
                    turn = 0;
                }

                lineNumber.set(283);
                powers.addHeading(turn); lineNumber.set(284);

                if (fieldCentric) {
                    //converting to field centric
                    powers.addVector(new Vector2d(forward, strafing).rotate(-relativeHeading));

                } else {
                    powers.addVector(new Vector2d(forward, strafing));
                } lineNumber.set(292);


                //driving settings
                if (gamepad1.back) {
                    headingOffset = roadRunnerPoseEstimate.getHeading();
                }

                if (gamepad1.start && gamepad1.dpad_up) {
                    fieldCentric = false;
                } else if (gamepad1.start && gamepad1.dpad_down) {
                    fieldCentric = true;
                }

                lineNumber.set(306);
                setDrivePower(powers); lineNumber.set(307);

                break;
        }

        roadRunnerPos.setValue(roadRunnerPoseEstimate); lineNumber.set(312);
        roadRunnerVel.setValue(roadRunnerPoseVelocity); lineNumber.set(313);


        lineNumber.set(318);
        motorPowerTelemetry.setValue(drive.getDrivePowers()); lineNumber.set(319);

    }

    @Override
    public TelemetryPacket dashboard(TelemetryPacket packet) {

        if (driveState == DriveState.FOLLOW_PATH) {
            packet.fieldOverlay().getOperations().addAll(this.packet.fieldOverlay().getOperations());
        } else {

            packet.put("x", drive.pose.position.x);
            packet.put("y", drive.pose.position.y);
            packet.put("heading (deg)", drive.pose.heading.toDouble()*180/Math.PI);

            packet.fieldOverlay().setStrokeWidth(1);
            packet.fieldOverlay().setStroke("#b33fb5");
//            DashboardUtil.drawFullPoseHistory(packet.fieldOverlay(), localizer.getPoseHistory());
//            DashboardUtil.drawRobot(packet.fieldOverlay(), roadRunnerPoseEstimate);

            //draws the last 200 points the robot was at

            packet.fieldOverlay().setStroke("#3F51B5");

            //draws the robots current position

            Vector2d robotVelocity = roadRunnerPoseVelocity.getVector2d();


            DashboardUtil.drawArrow(packet.fieldOverlay(), roadRunnerPoseEstimate.getVector2d(), roadRunnerPoseEstimate.getVector2d().plus(robotVelocity));

        }

        packet.fieldOverlay().setStrokeWidth(1);

//        packet.fieldOverlay().setStroke("#0a0a0f");//black
//        DashboardUtil.drawIntake(packet.fieldOverlay(), robotPos, slidePos);
//
//
//        packet.fieldOverlay().setStroke("#3F51B5");//blue
//        DashboardUtil.drawRobot(packet.fieldOverlay(), robotPos);

            packet.fieldOverlay().setStroke("#0a0a0f");//black
            DashboardUtil.drawIntake(packet.fieldOverlay(), roadRunnerPoseEstimate, intakeDistance);


            packet.fieldOverlay().setStroke("#3F51B5");//blue
            DashboardUtil.drawRobot(packet.fieldOverlay(), roadRunnerPoseEstimate);

        return packet;
    }

    public void setDriveState(DriveState driveState) {
        this.driveState = driveState;
    }

    public DriveState getDriveState() {
        return driveState;
    }

    public void followPath(Action path) {
        this.path = path;
        canvas = new Canvas();
        this.path.preview(canvas);
        followPath = true;
    }

    public boolean isFinished() {
        return !followPath;
    }

    public void setDrivePower(MotorPowers motorPowers) {
        setDrivePower(motorPowers.getNormalizedVoltages(voltage));
    }

    public void setDrivePower(List<Double> targetPowers) {
        for (int i = 0; i<4; i++) {
            if ((actualPowers.get(i) == 0 && targetPowers.get(i) != 0) || (actualPowers.get(i) != 0 && targetPowers.get(i) == 0) || (Math.abs(targetPowers.get(i)- actualPowers.get(i))>.1)) {
                int finalI = i;
                motorActions.get(i).setAndQueueAction(() -> {
                    drivetrainMotors.get(finalI).setPower(targetPowers.get(finalI));
                    actualPowers.set(finalI, targetPowers.get(finalI));
                });
            }
        }
    }

    public double getVoltage() {
        return updatedVoltage;
    }

    public Pose2d getPoseEstimate() {
        return roadRunnerPoseEstimate;
    }

    public Pose2d getIntakePoseEstimate() {
        return robotToIntakePos(roadRunnerPoseEstimate != null ? roadRunnerPoseEstimate : new Pose2d(0, 0, 0), intakeDistance);
    }

    public double getIntakeY() {
        return intakeY;
    }

    public void holdPoint(Pose2d targetHoldPoint) {
        this.targetHoldPoint = targetHoldPoint;
        holdPoint = true;
    }

    public void holdPoint() {
        holdPoint = true;
    }

    public void cancelHoldPoint() {
        holdPoint = false;
        setDrivePower(Arrays.asList(0.0, 0.0, 0.0, 0.0));
    }

    public Pose2d getHoldPointError() {
        return targetHoldPoint.minus(roadRunnerPoseEstimate);
    }

    public void stopFollowPath() {
        followPath = false;
    }

    public void stopMotors() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }
}
