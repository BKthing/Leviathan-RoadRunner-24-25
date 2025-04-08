package org.firstinspires.ftc.teamcode.teleops;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.reefsharklibrary.data.Rotation;
import com.reefsharklibrary.data.Vector2d;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autos.BlueRRLeft0plus7Auto;
import org.firstinspires.ftc.teamcode.subsystems.NewDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.NewIntake;
import org.firstinspires.ftc.teamcode.subsystems.NewOuttake;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

import java.util.Arrays;

@TeleOp
public class CameraPickupTest extends LinearOpMode {
    NewDrivetrain drivetrain;
    NewIntake intake;
    NewOuttake outtake;
    VisionSubsystem vision;
    MasterThread masterThread;
    Telemetry.Item loopTimeTelem;

    Boolean blueAlliance = true;

    double timeThreshold = -1;


    private Encoder verticalSlideEncoder, horizontalSlideEncoder;

    private TouchSensor breakBeam;

    private Telemetry.Item hasSampleTelem;

    private Telemetry.Item targetAngle;

    private Telemetry.Item sampleDistance;

    private double maxGrabAngle = Math.toRadians(193);
    private double minGrabAngle = Math.toRadians(167);

    private double extensionDistance = 0;

    private final ElapsedTimer autoTimer = new ElapsedTimer();

    private double loopTime = .05;

    private double prevExtensionError = 0;

    Vector2d holdPoint = new Vector2d(0, 0);

    double targetHeading = 0;
    double prevHeading = 0;
    ElapsedTimer loopTimer = new ElapsedTimer();

    Action grabFromSubmersible = new GrabFromSubmersible();

    boolean runAction = false;

    Action park;


    @Override
    public void runOpMode() throws InterruptedException {
        loopTimeTelem = telemetry.addData("Loop time:", loopTimer.milliSeconds());

        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);



        horizontalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalLeft"));
        breakBeam = hardwareMap.get(TouchSensor.class, "breakBeam");

        intake = new NewIntake(masterThread.getData(), horizontalSlideEncoder, breakBeam, blueAlliance, true, false, () -> drivetrain.getVoltage());

        verticalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalLeft"));

        outtake = new NewOuttake(masterThread.getData(), intake, verticalSlideEncoder, blueAlliance, true, true, true, false, () -> drivetrain.getVoltage());

        drivetrain = new NewDrivetrain(masterThread.getData(), outtake, intake);
        drivetrain.setDriveState(NewDrivetrain.DriveState.FOLLOW_PATH);


        vision = new VisionSubsystem(drivetrain, masterThread.getData(), blueAlliance, false);

        hasSampleTelem = telemetry.addData("Has sample", "");

        targetAngle = telemetry.addData("Target angle", "");

        sampleDistance = telemetry.addData("Sample dist", "");

        //its important that outtake is added after intake for update order purposes
        masterThread.addSubSystems(
                drivetrain,
                intake,
                outtake,
                vision
        );

        if (drivetrain.drive.pinpoint.fastIsPinpointCooked()) {
            throw new RuntimeException("pinpoint cooked");
        }

        waitForStart();
        masterThread.clearBulkCache();

//        drivetrain.drive.setPoseEstimate(new Pose2d(0, 60.23, Math.toRadians(180)));
        drivetrain.drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));


        while ( !isStopRequested()) {
            masterThread.unThreadedUpdate();

            sampleDistance.setValue(vision.getSampleRobotDiff().getMagnitude() - 9.59029 - intake.getIntakeHorizontalOffset() - intake.getActualSlidePos());


//            double targetHeading = prevHeading+Rotation.inRange((vision.getTargetRobotPose().getHeading()-prevHeading)*.125, Math.PI, -Math.PI);
//                drivetrain.holdPoint(holdPoint.toPose(targetHeading));//vision.getTargetRobotPose()
            if (gamepad1.a) {
                intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION);
                vision.recenter();
                drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(180)));
            } else if (gamepad1.y) {
                drivetrain.cancelHoldPoint();
                intake.toIntakeState(NewIntake.ToIntakeState.RETRACT);
                drivetrain.stopFollowPath();
            }

//            if (gamepad1.x) {
//                park = new Action() {
//                    boolean firstLoop = true;
//
//                    Action path;
//
//                    @Override
//                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                        if (firstLoop) {
//                            if (drivetrain.getPoseEstimate().getX()>40) {
//                                //park from bucket
//
//                                intake.toIntakeState(NewIntake.ToIntakeState.RETRACT_AND_STOP_INTAKING);
//
//                                path = drivetrain.drive.actionBuilder(MathUtil.toRoadRunnerPose(drivetrain.getPoseEstimate()))
//                                        .afterTime(.2, () -> {
//                                            outtake.toOuttakeState(NewOuttake.ToOuttakeState.TOUCH_BAR);
//                                        })
//                                        .splineToLinearHeading(new Pose2d(21.5, 8, Math.toRadians(180)), Math.toRadians(180), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(80))), new ProfileAccelConstraint(-27, 80))
//                                        .afterTime(0, () -> {
//                                            drivetrain.holdPoint(new com.reefsharklibrary.data.Pose2d(21.5, 8, Math.toRadians(180)));
//                                        })
//                                        .build();
//                            } else {
//                                //park from submersible
//
//                                intake.toIntakeState(NewIntake.ToIntakeState.RETRACT_AND_STOP_INTAKING);
//
//                                if (drivetrain.getPoseEstimate().getX()<27) {
//                                    double tangent = new com.reefsharklibrary.data.Vector2d(28, 8).minus(drivetrain.getPoseEstimate().getVector2d()).getDirection();
//
//                                    path = drivetrain.drive.actionBuilder(MathUtil.toRoadRunnerPose(drivetrain.getPoseEstimate()))
//                                            .setTangent(tangent)
//                                            .lineToXLinearHeading(26.5, Math.toRadians(180), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(80))), new ProfileAccelConstraint(-40, 80))
//                                            .afterTime(0, () -> {
//                                                outtake.toOuttakeState(NewOuttake.ToOuttakeState.TOUCH_BAR);
//                                            })
//                                            .setTangent(Math.toRadians(180))
//                                            .lineToX(21.5)
//                                            .afterTime(0, () -> {
//                                                drivetrain.holdPoint(new com.reefsharklibrary.data.Pose2d(21.5, 8, Math.toRadians(180)));
//                                            })
//                                            .build();
//                                } else {
//                                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.TOUCH_BAR);
//
//                                    double tangent = new com.reefsharklibrary.data.Vector2d(21, 8).minus(drivetrain.getPoseEstimate().getVector2d()).getDirection();
//
//                                    path = drivetrain.drive.actionBuilder(MathUtil.toRoadRunnerPose(drivetrain.getPoseEstimate()))
//                                            .setTangent(tangent)
//                                            .lineToXLinearHeading(21.5, Math.toRadians(180))
//                                            .afterTime(0, () -> {
//                                                drivetrain.holdPoint(new com.reefsharklibrary.data.Pose2d(21.5, 8, Math.toRadians(180)));
//                                            })
//                                            .build();
//                                }
//
//                            }
//
//                            firstLoop = false;
//                        }
//
//                        return path.run(telemetryPacket);
//                    }
//                };
//
//                drivetrain.followPath(park);
//            }


            if (gamepad1.b) {
                prevHeading = drivetrain.getPoseEstimate().getHeading();
                drivetrain.followPath(new GrabFromSubmersible());
//                grabFromSubmersible = new GrabFromSubmersible();
//                runAction = true;
                autoTimer.reset();
            }

//            if (runAction) {
//                runAction = grabFromSubmersible.run(new TelemetryPacket());
//            }

            loopTimeTelem.setValue(loopTimer.milliSeconds());

            loopTime = loopTimer.seconds();
            loopTimer.reset();
        }
    }

    public class GrabFromSubmersible implements Action {
        BlueRRLeft0plus7Auto.GrabFromSubmersibleState grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.SEARCHING;

        boolean searching = false;
        Action searchTurn;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (intake.getPrevIntakingState() == NewIntake.IntakingState.FINISH_EJECTING || intake.getPrevIntakingState() == NewIntake.IntakingState.START_EJECTING) {
                grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.EJECTING;
            }

            if (intake.getPrevIntakingState() == NewIntake.IntakingState.INTAKING_A_LITTLE_MORE || intake.getPrevIntakingState() == NewIntake.IntakingState.INTAKING_SPIN_OUT || intake.getPrevIntakingState() == NewIntake.IntakingState.FINISH_INTAKING) {
                timeThreshold = -1;
                drivetrain.cancelHoldPoint();
                return false;
            }

            switch (grabFromSubmersibleState) {
                case SEARCHING:
                    if (vision.hasSample()) {
                        targetHeading = MathUtil.clip(Rotation.inRange(prevHeading+Rotation.inRange((vision.getTargetRobotPose().getHeading()-prevHeading), Math.PI, -Math.PI)*1, 2*Math.PI, 0), minGrabAngle, maxGrabAngle);

                        drivetrain.holdPoint(holdPoint.toPose(targetHeading));

                        autoTimer.reset();
                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.APPROACHING_HEADING;
                    } else if (autoTimer.seconds()>.5) {
                        if (!searching) {
                            drivetrain.cancelHoldPoint();
                            searchTurn = drivetrain.drive.actionBuilder(MathUtil.toRoadRunnerPose(holdPoint.toPose(Math.toRadians(180))))
                                    .turnTo(maxGrabAngle, new TurnConstraints(.1 *Math.PI, -2 *Math.PI, 2 *Math.PI))
                                    .turnTo(minGrabAngle, new TurnConstraints(.1 *Math.PI, -2 *Math.PI, 2 *Math.PI))
                                    .build();
                            searching = true;
                        }
                        searching = searchTurn.run(telemetryPacket);
                    }
                    break;
                case APPROACHING_HEADING:
                    targetHeading = MathUtil.clip(Rotation.inRange(prevHeading+Rotation.inRange((vision.getTargetRobotPose().getHeading()-prevHeading), Math.PI, -Math.PI)*.15, 2*Math.PI, 0), minGrabAngle, maxGrabAngle);
                    drivetrain.holdPoint(holdPoint.toPose(targetHeading));

                    if (Math.abs(drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).getHeading())<Math.toRadians(20)) {
                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.APPROACHING;

                        extensionDistance = Math.max(vision.getSampleRobotDiff().getMagnitude() - 9.59029 - intake.getIntakeHorizontalOffset()-3.5, 2.5);//Math.max(extensionDistance+(vision.getSampleRobotDiff().getMagnitude() - 9.59029 - intake.getIntakeHorizontalOffset() - 4)*.125, 3);
                        prevExtensionError = 0;

                        intake.setTargetSlidePos(extensionDistance);
                    }
                    break;
                case APPROACHING:
                    double extensionError = (vision.getSampleRobotDiff().getMagnitude() - 9.59029 - intake.getIntakeHorizontalOffset() - intake.getTargetSlidePos() -3.5);

                    double extensionErrorVel = (extensionError-prevExtensionError)/loopTime;

                    extensionDistance = Math.max(extensionDistance + .15*extensionError - .0*extensionErrorVel, 2.5);//Math.max(extensionDistance+(vision.getSampleRobotDiff().getMagnitude() - 9.59029 - intake.getIntakeHorizontalOffset() - 4)*.125, 3);
                    intake.setTargetSlidePos(extensionDistance);

                    prevExtensionError = extensionError;

                    //adjust holdPoint heading
                    targetHeading = MathUtil.clip(Rotation.inRange(prevHeading+Rotation.inRange((vision.getTargetRobotPose().getHeading()-prevHeading), Math.PI, -Math.PI)*.15, 2*Math.PI, 0), minGrabAngle, maxGrabAngle);

                    drivetrain.holdPoint(holdPoint.toPose(targetHeading));//, maxGrabAngle

                    if (drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(1, 1, Math.toRadians(3.5)))
                        && drivetrain.getPoseVelocity().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(.5, .5, Math.toRadians(.5))) && Math.abs(extensionError)<.8 && Math.abs(extensionErrorVel)<.5) {
                        extensionDistance = intake.getTargetSlidePos()+3;
                        intake.setTargetSlidePos(extensionDistance);
                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.MOVING_FORWARD;

                        autoTimer.reset();
                    }
                    break;
                case MOVING_FORWARD:
                    if (autoTimer.seconds()>.3) {
                        intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                        intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                        autoTimer.reset();
                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.INTAKING_1;
                    }
                    break;
                case INTAKING_1:
//hello brett king
                    if (autoTimer.seconds()>.4) {
                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.RETRACTING;
                        autoTimer.reset();
                    }
//                    else {
//                        targetHeading = MathUtil.clip(Rotation.inRange(prevHeading + Rotation.inRange((vision.getTargetRobotPose().getHeading() - prevHeading), Math.PI, -Math.PI) * .15, 2 * Math.PI, 0), minGrabAngle, maxGrabAngle);
//
//                        drivetrain.holdPoint(holdPoint.toPose(targetHeading));
//
//                        extensionDistance = MathUtil.clip(extensionDistance + 12 * loopTime, -.5, 18.5);
//                        intake.setTargetSlidePos(extensionDistance);
//
//                    }
                    break;
                case RETRACTING:

                    if (autoTimer.seconds()>.2 || extensionDistance == 2) {
                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.INTAKING_2;
                    } else {
                        targetHeading = MathUtil.clip(Rotation.inRange(prevHeading + Rotation.inRange((vision.getTargetRobotPose().getHeading() - prevHeading), Math.PI, -Math.PI) * .15, 2 * Math.PI, 0), minGrabAngle, maxGrabAngle);

                        drivetrain.holdPoint(holdPoint.toPose(targetHeading));

                        extensionDistance = MathUtil.clip(extensionDistance - 8 * loopTime, 2, 18.5);
                        intake.setTargetSlidePos(extensionDistance);

                    }
                    break;
                case INTAKING_2:

//                    targetHeading = MathUtil.clip(Rotation.inRange(prevHeading + Rotation.inRange((vision.getTargetRobotPose().getHeading() - prevHeading), Math.PI, -Math.PI) * .15, 2 * Math.PI, 0), minGrabAngle, maxGrabAngle);

//                    drivetrain.holdPoint(holdPoint.toPose(targetHeading));

                    if (autoTimer.seconds() > 2) {
                        intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION);
                        double curHeading = drivetrain.getPoseEstimate().getHeading();

                        if (curHeading > Math.toRadians(183)) {
                            targetHeading = Math.toRadians(176);
                            drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(176)));
                        } else if (curHeading < Math.toRadians(177)) {
                            targetHeading = Math.toRadians(183);
                            drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(185)));
                        } else {
                            targetHeading = Math.toRadians(175);
                            drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(175)));
                        }

                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.RESETTING;
                    } else {
                        extensionDistance = MathUtil.clip(extensionDistance + 6 * loopTime, -.5, 18.5);
                        intake.setTargetSlidePos(extensionDistance);
                    }

                    break;
                case EJECTING:
                    if (intake.getPrevIntakingState() == NewIntake.IntakingState.IDLE) {

                        intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION);
                        extensionDistance = 1;
                        drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(180)));

                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.RESETTING;
                    }
                    break;
                case RESETTING:
                    if (drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(1, 1, Math.toRadians(3.5)))) {
                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.SEARCHING;
                        extensionDistance = 1;
                        autoTimer.reset();
                    }
                    break;
            }

            prevHeading = targetHeading;

            return true;


        }
    }

}
