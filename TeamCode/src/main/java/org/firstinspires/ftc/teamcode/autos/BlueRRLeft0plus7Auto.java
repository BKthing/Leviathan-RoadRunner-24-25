package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive3.PARAMS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.reefsharklibrary.data.Rotation;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.NewDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.NewIntake;
import org.firstinspires.ftc.teamcode.subsystems.NewOuttake;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

import java.util.Arrays;

@Autonomous
public class BlueRRLeft0plus7Auto extends LinearOpMode {

    public enum GrabFromSubmersibleState {
        SEARCHING,
        APPROACHING_HEADING,
        APPROACHING,
        INTAKING,
        EJECTING,
        RESETTING
    }

    private double targetHeading = Math.toRadians(180);
    private double prevHeading = Math.toRadians(180);

    private final double maxGrabAngle = Math.toRadians(193);
    private final double minGrabAngle = Math.toRadians(167);

    NewDrivetrain drivetrain;
    NewIntake intake;
    NewOuttake outtake;
    VisionSubsystem vision;
    MasterThread masterThread;
    Telemetry.Item loopTimeTelem;

    Boolean blueAlliance = true;

    private Encoder verticalSlideEncoder, horizontalSlideEncoder;

    private TouchSensor breakBeam;

    double extensionDistance = 0;

    private final com.reefsharklibrary.data.Vector2d holdPoint = new com.reefsharklibrary.data.Vector2d(23, 8);

    private final ElapsedTimer loopTimer = new ElapsedTimer();

    private final ElapsedTimer autoTimer = new ElapsedTimer();

    private final ElapsedTimer totalAutoTimer = new ElapsedTimer();

    double timeThreshold = -1;

    double loopTime = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        loopTimeTelem = telemetry.addData("Loop time:", loopTimer.milliSeconds());

        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);

        horizontalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalLeft"));
        breakBeam = hardwareMap.get(TouchSensor.class, "breakBeam");

        verticalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalLeft"));

        intake = new NewIntake(masterThread.getData(), horizontalSlideEncoder, breakBeam, blueAlliance, false, true, () -> drivetrain.getVoltage());

        outtake = new NewOuttake(masterThread.getData(), intake, verticalSlideEncoder, blueAlliance, false, true, true, true, () -> drivetrain.getVoltage());

        drivetrain = new NewDrivetrain(masterThread.getData(), outtake, intake);
        drivetrain.setDriveState(NewDrivetrain.DriveState.FOLLOW_PATH);


        vision = new VisionSubsystem(drivetrain, masterThread.getData(), blueAlliance);

        //its important that outtake is added after intake for update order purposes
        masterThread.addSubSystems(
                drivetrain,
                intake,
                outtake,
                vision
        );

        Action intakeBlock = new IntakeBlock();


        Action preload = drivetrain.drive.actionBuilder(new Pose2d(38.93, 60.23, Math.toRadians(180)))
                .setTangent(Math.toRadians(310))
                .afterTime(.3, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
                })
                .afterTime(.8, () -> {//0
                    intake.setTargetSlidePos(13.5);
                    extensionDistance = 13.5;
                })
                .splineToLinearHeading(new Pose2d(62, 55, Math.toRadians(250)), Math.toRadians(0))
                .build();

        Action moveToGrabBlock1 = drivetrain.drive.actionBuilder(new Pose2d(62, 55, Math.toRadians(250)))
                .setTangent(Math.toRadians(250))
                .afterTime(0, () -> {
//                    intake.setTargetSlidePos(9);
//                    extensionDistance = 9;
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                    autoTimer.reset();
                })
                .splineToConstantHeading(new Vector2d(57, 51), Math.toRadians(253))
//                .splineToLinearHeading(new Pose2d(50.5, 50, Math.toRadians(270)), Math.toRadians(270))
                .build();

        Action moveToScoreBlock1 = drivetrain.drive.actionBuilder(new Pose2d(57, 51, Math.toRadians(253)))
                .setTangent(Math.toRadians(84))
                .splineToLinearHeading(new Pose2d(62, 53.5, Math.toRadians(265)), Math.toRadians(85))
                .build();

        Action moveToGrabBlock2 = drivetrain.drive.actionBuilder(new Pose2d(62, 53.5, Math.toRadians(265)))
                .afterTime(0, () -> {
                    intake.setTargetSlidePos(10);
                    extensionDistance = 10;
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                    autoTimer.reset();
                })
                .waitSeconds(.1)
                .setTangent(265)
                .splineToConstantHeading(new Vector2d(61, 51), Math.toRadians(265))
                .build();

        Action moveToScoreBlock2 = drivetrain.drive.actionBuilder(new Pose2d(61, 51, Math.toRadians(265)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(63, 54, Math.toRadians(270)), Math.toRadians(90))
                .build();

        Action moveToGrabBlock3 = drivetrain.drive.actionBuilder(new Pose2d(63, 54, Math.toRadians(270)))
                .afterTime(0, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                })
                .afterTime(.3, () -> {
                    intake.setTargetSlidePos(11.5);
                    extensionDistance = 11.5;
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                    autoTimer.reset();
                })
                .waitSeconds(.1)
                .setTangent(270)
                .splineToLinearHeading(new Pose2d(62.5, 51.5, Math.toRadians(281)), Math.toRadians(270))
                .build();

        Action moveToScoreBlock3 = drivetrain.drive.actionBuilder(new Pose2d(62.5, 51.5, Math.toRadians(281)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(62.5, 54.5, Math.toRadians(270)), Math.toRadians(90))
                .build();

        Action moveToSubmersible1 = new Action() {
            boolean notCanceled = true;

            final Action action = drivetrain.drive.actionBuilder(new Pose2d(59.5, 54.5, Math.toRadians(240)))
                    .setTangent(Math.toRadians(245))
                    .afterTime(0, () -> {
                        timeThreshold = 2.5;
                    })
                    .afterDisp(37, () -> {
                        intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION);
                        extensionDistance = 3;
                        prevHeading = drivetrain.getPoseEstimate().getHeading();
                        drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(180)));
                        autoTimer.reset();
                        notCanceled = false;
                    })
                    .lineToYLinearHeading(0, Math.toRadians(190), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-60, PARAMS.maxProfileAccel))
                    .build();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return notCanceled && action.run(telemetryPacket);
            }
        };

        Action moveToSubmersible2 = new Action() {
            boolean notCanceled = true;

            final Action action = drivetrain.drive.actionBuilder(new Pose2d(59.5, 55, Math.toRadians(240)))
                    .setTangent(Math.toRadians(245))
                    .afterTime(0, () -> {
                        timeThreshold = 2.5;
                    })
                    .afterDisp(37, () -> {
                        intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION);
                        extensionDistance = 3;
                        prevHeading = drivetrain.getPoseEstimate().getHeading();
                        drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(180)));
                        autoTimer.reset();
                        notCanceled = false;
                    })
                    .lineToYLinearHeading(0, Math.toRadians(190), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-60, PARAMS.maxProfileAccel))
                    .build();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return notCanceled && action.run(telemetryPacket);
            }
        };

        Action moveToSubmersible3 = new Action() {
            boolean notCanceled = true;

            final Action action = drivetrain.drive.actionBuilder(new Pose2d(59.5, 55, Math.toRadians(240)))
                    .setTangent(Math.toRadians(245))
                    .afterTime(0, () -> {
                        timeThreshold = 2.5;
                    })
                    .afterDisp(37, () -> {
                        intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION);
                        extensionDistance = 3;
                        prevHeading = drivetrain.getPoseEstimate().getHeading();
                        drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(180)));
                        autoTimer.reset();
                        notCanceled = false;
                    })
                    .lineToYLinearHeading(0, Math.toRadians(190), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-60, PARAMS.maxProfileAccel))
                    .build();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return notCanceled && action.run(telemetryPacket);
            }
        };


        Action moveToScoreFromSubmersible1 = drivetrain.drive.actionBuilder(new Pose2d(holdPoint.getX(), holdPoint.getY(), Math.toRadians(180)))
                .setTangent(Math.toRadians(0))
                .afterTime(0, () -> {
                    drivetrain.cancelHoldPoint();
                })
                .splineTo(new Vector2d(59.5, 55), Math.toRadians(60), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-35, PARAMS.maxProfileAccel))
                .build();

        Action moveToScoreFromSubmersible2 = drivetrain.drive.actionBuilder(new Pose2d(holdPoint.getX(), holdPoint.getY(), Math.toRadians(180)))
                .setTangent(Math.toRadians(0))
                .afterTime(0, () -> {
                    drivetrain.cancelHoldPoint();
                })
                .splineTo(new Vector2d(59.5, 55), Math.toRadians(60), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-35, PARAMS.maxProfileAccel))
                .build();

        Action moveToScoreFromSubmersible3 = drivetrain.drive.actionBuilder(new Pose2d(holdPoint.getX(), holdPoint.getY(), Math.toRadians(180)))
                .setTangent(Math.toRadians(0))
                .afterTime(0, () -> {
                    drivetrain.cancelHoldPoint();
                })
                .splineTo(new Vector2d(59.5, 55), Math.toRadians(60), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-35, PARAMS.maxProfileAccel))
                .build();


        Action park = new Action() {
            boolean firstLoop = true;

            Action path;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (firstLoop) {
                    if (drivetrain.getPoseEstimate().getX()>29) {
                        //park from bucket

                        intake.toIntakeState(NewIntake.ToIntakeState.RETRACT_AND_STOP_INTAKING);

                        path = drivetrain.drive.actionBuilder(MathUtil.toRoadRunnerPose(drivetrain.getPoseEstimate()))
                                .afterTime(.4, () -> {
                                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.TOUCH_BAR);
                                })
                                .splineToLinearHeading(new Pose2d(22, 8, Math.toRadians(180)), Math.toRadians(180))
                                .build();
                    } else {
                        //park from submersible

                        intake.toIntakeState(NewIntake.ToIntakeState.RETRACT_AND_STOP_INTAKING);

                        if (Math.abs(drivetrain.getPoseEstimate().getX()-28)>1) {
                            double tangent = new com.reefsharklibrary.data.Vector2d(28, 8).minus(drivetrain.getPoseEstimate().getVector2d()).getDirection();

                            path = drivetrain.drive.actionBuilder(MathUtil.toRoadRunnerPose(drivetrain.getPoseEstimate()))
                                    .setTangent(tangent)
                                    .lineToXLinearHeading(28, Math.toRadians(180))
                                    .afterTime(0, () -> {
                                        outtake.toOuttakeState(NewOuttake.ToOuttakeState.TOUCH_BAR);
                                    })
                                    .lineToX(22)
                                    .build();
                        } else {
                            outtake.toOuttakeState(NewOuttake.ToOuttakeState.TOUCH_BAR);

                            double tangent = new com.reefsharklibrary.data.Vector2d(22, 8).minus(drivetrain.getPoseEstimate().getVector2d()).getDirection();

                            path = drivetrain.drive.actionBuilder(MathUtil.toRoadRunnerPose(drivetrain.getPoseEstimate()))
                                    .setTangent(tangent)
                                    .lineToXLinearHeading(28, Math.toRadians(180))
                                    .build();
                        }

                    }

                    firstLoop = false;
                }

                return path.run(telemetryPacket);
            }
        };

        drivetrain.drive.setPoseEstimate(new Pose2d(1, 1, 0));
        drivetrain.drive.pinpoint.update();

        if (drivetrain.drive.pinpoint.fastIsPinpointCooked()) {
            throw new RuntimeException("pinpoint cooked");
        }

        waitForStart();

        totalAutoTimer.reset();

        drivetrain.drive.setPoseEstimate(new Pose2d(38.93, 60.23, Math.toRadians(180)));

        masterThread.clearBulkCache();

        intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
        outtake.toOuttakeState(NewOuttake.ToOuttakeState.PLACE_BEHIND);

        drivetrain.followPath(new SequentialAction(
                new RunToTimeThreshold(
                        new SequentialAction(
                                preload,
                                new ScoreBlock(),
                                moveToGrabBlock1,
                                intakeBlock,
                                moveToScoreBlock1,
                                new ScoreBlock(),
                                moveToGrabBlock2,
                                intakeBlock,
                                moveToScoreBlock2,
                                new ScoreBlock(),
                                moveToGrabBlock3,
                                intakeBlock,
                                moveToScoreBlock3,
                                new ScoreBlock(),

                                moveToSubmersible1,
                                new GrabFromSubmersible(),
                                moveToScoreFromSubmersible1,
                                new ScoreBlock(),

                                moveToSubmersible2,
                                new GrabFromSubmersible(),
                                moveToScoreFromSubmersible2,
                                new ScoreBlock(),

                                moveToSubmersible3,
                                new GrabFromSubmersible(),
                                moveToScoreFromSubmersible3,
                                new ScoreBlock()
                        )
                ),
                park
        ));

        while ( !isStopRequested()) {

            masterThread.unThreadedUpdate();

            loopTime = loopTimer.seconds();
            loopTimeTelem.setValue(loopTimer.milliSeconds());

            loopTimer.reset();
        }

//        intake.overrideSpinOut();
//
//        drivetrain.stopMotors();
//        intake.stopMotors();
//        outtake.stopMotors();
//
//        sleep(250);
    }

    public class RunToTimeThreshold implements Action {
        Action action;

        public RunToTimeThreshold(Action action) {
            this.action = action;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return action.run(telemetryPacket) && ((30-totalAutoTimer.seconds())>timeThreshold);
        }
    }

    public class IntakeBlock implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (intake.getPrevIntakingState() != NewIntake.IntakingState.INTAKING && intake.getPrevIntakingState() != NewIntake.IntakingState.START_INTAKING && intake.getPrevIntakingState() != NewIntake.IntakingState.SERVO_STALL_START_UNJAMMING && intake.getPrevIntakingState() != NewIntake.IntakingState.SERVO_STALL_UNJAMMING_SPIN_OUT) {
                return false;
            } else if (autoTimer.seconds()>2) {
                intake.toIntakeState(NewIntake.ToIntakeState.RETRACT);
                return false;
            } else {
                extensionDistance = MathUtil.clip(extensionDistance + 20 * loopTime, -.5, 18.5);
                intake.setTargetSlidePos(extensionDistance);
                return true;
            }
        }
    }

    public class ScoreBlock implements Action {
        private boolean firstLoop = true;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstLoop) {
                if (!intake.isBreakBeam() && (intake.getPrevIntakeState() == NewIntake.IntakeState.RETRACTING_INTAKE || intake.getPrevIntakeState() == NewIntake.IntakeState.RETRACTING || intake.getPrevIntakeState() == NewIntake.IntakeState.WAITING_AFTER_RETRACTING || intake.getPrevIntakeState() == NewIntake.IntakeState.WAITING_FOR_TRANSFER)) {
                    intake.setIntakingState(NewIntake.IntakingState.START_EJECTING_PARTIAL_GRAB);
                    return false;
                } else {
                    firstLoop = false;
                    autoTimer.reset();
                    return true;
                }
            } else {
                if (outtake.getOuttakeState() == NewOuttake.OuttakeState.WAITING_PLACE_BEHIND || outtake.getFailedToTransfer()) {// || intake.getPrevIntakingState() == NewIntake.IntakingState.INTAKING
                    outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);
                    return false;
                } else {
                    return true;
                }
            }
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
                        targetHeading = MathUtil.clip(Rotation.inRange(prevHeading+Rotation.inRange((vision.getTargetRobotPose().getHeading()-prevHeading), Math.PI, -Math.PI)*.3, 2*Math.PI, 0), minGrabAngle, maxGrabAngle);

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

                    if (Math.abs(drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).getHeading())<Math.toRadians(5)) {
                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.APPROACHING;

                        extensionDistance = Math.max(vision.getSampleRobotDiff().getMagnitude() - 9.59029 - intake.getIntakeHorizontalOffset() - intake.getActualSlidePos() - 4.8, 2.5);//Math.max(extensionDistance+(vision.getSampleRobotDiff().getMagnitude() - 9.59029 - intake.getIntakeHorizontalOffset() - 4)*.125, 3);
                        intake.setTargetSlidePos(extensionDistance);
                    }
                    break;
                case APPROACHING:
                    //adjustSlides
//                        double newExtensionDistance = Math.max(extensionDistance+(vision.getSampleRobotDiff().getMagnitude() - 9.59029 - intake.getIntakeHorizontalOffset() - 4)*.125, 3);
//                        if (Math.abs(newExtensionDistance - extensionDistance) > .1) {
//                            extensionDistance = newExtensionDistance;
//                            intake.setTargetSlidePos(extensionDistance);
//                        }

                    //adjust holdPoint heading
                    targetHeading = MathUtil.clip(Rotation.inRange(prevHeading+Rotation.inRange((vision.getTargetRobotPose().getHeading()-prevHeading), Math.PI, -Math.PI)*.15, 2*Math.PI, 0), minGrabAngle, maxGrabAngle);

                    drivetrain.holdPoint(holdPoint.toPose(targetHeading));//, maxGrabAngle


                    if (drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(1, 1, Math.toRadians(3.5)))) {
                        intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                        intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.INTAKING;
                    }
                    break;
                case INTAKING:
//                    if (intake.getPrevIntakingState() != NewIntake.IntakingState.INTAKING && intake.getPrevIntakingState() != NewIntake.IntakingState.START_INTAKING && intake.getPrevIntakingState() != NewIntake.IntakingState.SERVO_STALL_START_UNJAMMING && intake.getPrevIntakingState() != NewIntake.IntakingState.SERVO_STALL_UNJAMMING_SPIN_OUT) {
//                        return false;
//                    }

                    targetHeading = MathUtil.clip(Rotation.inRange(prevHeading+Rotation.inRange((vision.getTargetRobotPose().getHeading()-prevHeading), Math.PI, -Math.PI)*.15, 2*Math.PI, 0), minGrabAngle, maxGrabAngle);

                    drivetrain.holdPoint(holdPoint.toPose(targetHeading));

                    if (autoTimer.seconds() > 2.5) {
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
                        extensionDistance = MathUtil.clip(extensionDistance + 8 * loopTime, -.5, 18.5);
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
