package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive3.PARAMS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
public class RRRight6plus0Auto extends LinearOpMode {
    NewDrivetrain drivetrain;
    NewIntake intake;
    NewOuttake outtake;
    MasterThread masterThread;
    Telemetry.Item loopTimeTelem;

    Boolean blueAlliance = true;

    ElapsedTimer autoTimer = new ElapsedTimer();

    double loopTime = .05;

    double timeThreshold = 12.5;

    double targetHeading = Math.toRadians(270);

    double prevHeading = targetHeading;

    double maxGrabAngle = Math.toRadians(285);
    double minGrabAngle = Math.toRadians(255);

    boolean grabbedYellow = false;

    VisionSubsystem vision;


    com.reefsharklibrary.data.Vector2d holdPoint = new com.reefsharklibrary.data.Vector2d(-4, 33);

    double extensionDistance = 0;

    private Encoder verticalSlideEncoder, horizontalSlideEncoder;

    private TouchSensor breakBeam;

    private final ElapsedTimer totalAutoTimer = new ElapsedTimer();

    private boolean skipDrop = false;

    private Telemetry.Item skipDropTelem;



    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTimer loopTimer = new ElapsedTimer();
        loopTimeTelem = telemetry.addData("Loop time:", loopTimer.milliSeconds());

        horizontalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalLeft"));
        breakBeam = hardwareMap.get(TouchSensor.class, "breakBeam");

        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);

        intake = new NewIntake(masterThread.getData(), horizontalSlideEncoder, breakBeam, blueAlliance, false, true, () -> drivetrain.getVoltage());

        verticalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalLeft"));

        outtake = new NewOuttake(masterThread.getData(), intake, verticalSlideEncoder, blueAlliance, false, true, true, true, () -> drivetrain.getVoltage());


        drivetrain = new NewDrivetrain(masterThread.getData(), outtake, intake);
        drivetrain.setDriveState(NewDrivetrain.DriveState.FOLLOW_PATH);

        vision = new VisionSubsystem(drivetrain, masterThread.getData(), blueAlliance, true);

        skipDropTelem = telemetry.addData("Skip Drop", skipDrop);

        //its important that outtake is added after intake for update order purposes
        masterThread.addSubSystems(
                drivetrain,
                intake,
                outtake,
                vision
        );





        Action preload = new Action() {
            boolean notCanceled = true;

            final Action action = drivetrain.drive.actionBuilder(new Pose2d(-5.5, 62.1, Math.toRadians(270)))
                    .setTangent(Math.toRadians(270))
                    .afterTime(.3, () -> {
                        intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
                    })
                    .afterTime(1.25, () -> {
                        outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);
                        notCanceled = false;
                    })
                    .splineToConstantHeading(new Vector2d(-5.5, 30.5), Math.toRadians(270), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-45, 50))
                    .build();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return notCanceled && action.run(telemetryPacket);
            }
        };

        Action moveToGrabBlock1 = drivetrain.drive.actionBuilder(new Pose2d(-5.5, 30.5, Math.toRadians(270)))
                .setTangent(Math.toRadians(90))
                .afterDisp(20, () -> {
                    intake.setTargetSlidePos(18.5);
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE_AUTO_SHOVE_HEIGHT);
                })
                .splineToConstantHeading(new Vector2d(-10, 37), Math.toRadians(180), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-45, 65))
                .splineToSplineHeading(new Pose2d(-25.5, 40.5, Math.toRadians(235)), Math.toRadians(180))
                .turn(Math.toRadians(-75), new TurnConstraints(4 *Math.PI, -2 *Math.PI, 3 *Math.PI))//4, -2, 3
                .afterTime(.1, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
                    intake.setTargetSlidePos(18.5);
                })

                .setTangent(new com.reefsharklibrary.data.Vector2d(-35.5, 41).minus(new com.reefsharklibrary.data.Vector2d(-25.5, 40.5)).getDirection())
                .lineToXSplineHeading(-35.5, Math.toRadians(228))
                .build();

        Action moveToPlaceBlock2 = drivetrain.drive.actionBuilder(new Pose2d(-36.5, 41, Math.toRadians(231)))
                .afterTime(0, () -> {
                    intake.setTargetSlidePos(18.5);
                })
                .waitSeconds(.3)
                .turn(Math.toRadians(-71), new TurnConstraints(4 *Math.PI, -2 *Math.PI, 3 *Math.PI))
                .afterTime(.1, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.RETRACT);
                })
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-57.5, 12.5, Math.toRadians(270)), Math.toRadians(180), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-20, PARAMS.maxProfileAccel))
                .splineToConstantHeading(new Vector2d(-58.5, 15.5), Math.toRadians(90), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(35))), new ProfileAccelConstraint(-20, PARAMS.maxProfileAccel))
                .lineToYConstantHeading(48, new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-30, PARAMS.maxProfileAccel))
                .splineToConstantHeading(new Vector2d(-55.5,60.9), Math.toRadians(80), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-30, PARAMS.maxProfileAccel))
                .build();

        MinVelConstraint moveToScoreVelConstraint = new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(120)));
        ProfileAccelConstraint moveToScoreAccelConstraint = new ProfileAccelConstraint(-65, 80);

        MinVelConstraint firstMoveToGrabVelConstraint = new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(70)));
        ProfileAccelConstraint firstMoveToGrabAccelConstraint = new ProfileAccelConstraint(-30, 60);

        MinVelConstraint moveToGrabVelConstraint = new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(70)));
        ProfileAccelConstraint moveToGrabAccelConstraint = new ProfileAccelConstraint(-35, 60);

        Action moveToScoreSpecimen2 = drivetrain.drive.actionBuilder(new Pose2d(-55.5, 60.9, Math.toRadians(270)))
                .waitSeconds(.2)
                .setTangent(Math.toRadians(305))
                .splineToConstantHeading(new Vector2d(-3.5, 29.5), Math.toRadians(300), moveToScoreVelConstraint, moveToScoreAccelConstraint)

                .build();


        Action moveToGrabSpecimen3 = drivetrain.drive.actionBuilder(new Pose2d(-4, 29.5, Math.toRadians(270)))
                .setTangent(Math.toRadians(115))
                .afterTime(.75, () -> {
                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.WAIT_DROP_BEHIND);
                })
                .splineToConstantHeading(new Vector2d(-37, 59), Math.toRadians(90), firstMoveToGrabVelConstraint, firstMoveToGrabAccelConstraint)
                .afterTime(.1, () -> {
                    outtake.toClawPosition(NewOuttake.ClawPosition.CLOSED);
                })
                .lineToY(60.3, moveToGrabVelConstraint, moveToGrabAccelConstraint)
                .waitSeconds(.2)
                .build();

        Action moveToScoreSpecimen3 = drivetrain.drive.actionBuilder(new Pose2d(-37, 60.3, Math.toRadians(270)))
                .setTangent(new com.reefsharklibrary.data.Vector2d(-4, 29.5).minus(new com.reefsharklibrary.data.Vector2d(-37, 60.7)).getDirection())
                .lineToY(29.5, moveToScoreVelConstraint, moveToScoreAccelConstraint)
                .afterTime(0, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION);
                    extensionDistance = 3;
                })
                .build();

        Action moveToDropSample3 = drivetrain.drive.actionBuilder(new Pose2d(-4, 33, Math.toRadians(270)))
                .setTangent(Math.toRadians(115))
                .afterTime(.6, () -> {
                    if (skipDrop) {
                        outtake.toOuttakeState(NewOuttake.ToOuttakeState.WAIT_DROP_BEHIND);
                        intake.setIntakingState(NewIntake.IntakingState.START_EJECTING_PARTIAL_GRAB);
                    }
                })
                .splineToConstantHeading(new Vector2d(-37, 57), Math.toRadians(90), moveToGrabVelConstraint, moveToGrabAccelConstraint)
                .build();

        Action moveToGrabSpecimen4 = drivetrain.drive.actionBuilder(new Pose2d(-37, 57, Math.toRadians(270)))
                .lineToY(60.3)
                .afterTime(.0, () -> {
                    outtake.toClawPosition(NewOuttake.ClawPosition.CLOSED);
                })
                .build();

        Action moveToScoreSpecimen4 = drivetrain.drive.actionBuilder(new Pose2d(-37, 60.3, Math.toRadians(270)))
                .waitSeconds(.1)
                .setTangent(new com.reefsharklibrary.data.Vector2d(-4, 29.5).minus(new com.reefsharklibrary.data.Vector2d(-37, 60.7)).getDirection())
                .afterTime(.2, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.RETRACT_AND_STOP_INTAKING);
                })
                .lineToY(29.5, moveToScoreVelConstraint, moveToScoreAccelConstraint)
                .build();

        Action moveToGrabSpecimen5 = drivetrain.drive.actionBuilder(new Pose2d(-3.5, 29.5, Math.toRadians(270)))
//                .setTangent(Math.toRadians(90))
//                .setTangent(new com.reefsharklibrary.data.Vector2d(-37, 61.7).minus(new com.reefsharklibrary.data.Vector2d(-3.5, 31)).getDirection())
                .afterTime(.75, () -> {
                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.WAIT_DROP_BEHIND);
                })
                .setTangent(Math.toRadians(115))
//                .lineToY(61.7, new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-35, PARAMS.maxProfileAccel))
                .splineToConstantHeading(new Vector2d(-37, 59), Math.toRadians(90), firstMoveToGrabVelConstraint, firstMoveToGrabAccelConstraint)
                .afterTime(.1, () -> {
                    outtake.toClawPosition(NewOuttake.ClawPosition.CLOSED);
                })
                .lineToY(60.3, moveToGrabVelConstraint, moveToGrabAccelConstraint)
                .waitSeconds(.2)
                .build();

        Action moveToScoreSpecimen5 = drivetrain.drive.actionBuilder(new Pose2d(-37, 60.3, Math.toRadians(270)))
                .setTangent(new com.reefsharklibrary.data.Vector2d(-4, 29.5).minus(new com.reefsharklibrary.data.Vector2d(-37, 61.7)).getDirection())
                .lineToY(29.5, moveToScoreVelConstraint, moveToScoreAccelConstraint)
//                .setTangent(Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(-4, 29), Math.toRadians(305), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-50, PARAMS.maxProfileAccel))
                .build();

        Action moveToGrabSpecimen6 = drivetrain.drive.actionBuilder(new Pose2d(-4, 29.5, Math.toRadians(270)))
                .setTangent(Math.toRadians(115))
                .splineToConstantHeading(new Vector2d(-37, 59), Math.toRadians(90), firstMoveToGrabVelConstraint, firstMoveToGrabAccelConstraint)
                .afterTime(.1, () -> {
                    outtake.toClawPosition(NewOuttake.ClawPosition.CLOSED);
                })
                .lineToY(60.3, moveToGrabVelConstraint, moveToGrabAccelConstraint)
                .waitSeconds(.2)
                .build();

        Action moveToScoreSpecimen6 = drivetrain.drive.actionBuilder(new Pose2d(-37, 60.3, Math.toRadians(270)))
                .setTangent(new com.reefsharklibrary.data.Vector2d(-4, 29.5).minus(new com.reefsharklibrary.data.Vector2d(-37, 60.7)).getDirection())
                .lineToY(29.5, moveToScoreVelConstraint, moveToScoreAccelConstraint)
                .build();

        Action park = drivetrain.drive.actionBuilder(new Pose2d(-4, 29.5, Math.toRadians(270)))
                .afterTime(.6, () -> {
                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.RETRACT_FROM_PLACE_BEHIND);
                })
                .afterTime(1.2, () -> {
                    intake.setTargetSlidePos(15);
                    extensionDistance = 15;
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                    autoTimer.reset();
                    outtake.outtakeState(NewOuttake.OuttakeState.IDLE);
                })
                .splineToLinearHeading(new Pose2d(-11.5, 56.5, Math.toRadians(175)), Math.toRadians(90)).build();

        drivetrain.drive.setPoseEstimate(new Pose2d(1, 1, 0));
        drivetrain.drive.pinpoint.update();

        if (drivetrain.drive.pinpoint.fastIsPinpointCooked()) {
            throw new RuntimeException("pinpoint cooked");
        }

        waitForStart();

        totalAutoTimer.reset();

        drivetrain.drive.setPoseEstimate(new  Pose2d(-5.5, 62.1, Math.toRadians(270)));

        masterThread.clearBulkCache();

        intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
        outtake.toOuttakeState(NewOuttake.ToOuttakeState.FIRST_PLACE_FRONT);

        drivetrain.followPath(new SequentialAction(
                preload,
                moveToGrabBlock1,
                new InstantAction(() -> intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE_AUTO_SHOVE_HEIGHT)),
                moveToPlaceBlock2,
                new InstantAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.CLOSED)),
                moveToScoreSpecimen2,
                new InstantAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.OPEN)),
                moveToGrabSpecimen3,
                moveToScoreSpecimen3,
                new InstantAction(() -> {
                    outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);
                    outtake.outtakeState(NewOuttake.OuttakeState.IDLE);
                    outtake.setSpecimenDropBehind(true);
                    drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(270)));


                    autoTimer.reset();
                }),
                new SleepAction(.2),
                new ParallelAction(
                        new RunToTimeThreshold(
                                new GrabFromSubmersible()
                        ),
                        new SequentialAction(
                                new SleepAction(.6),
                                new InstantAction(() -> {
                                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.RETRACT_FROM_PLACE_BEHIND);
                                })
                        )
                ),
                new InstantAction(
                        () -> {
                            drivetrain.cancelHoldPoint();
                        }
                ),
                new ParallelAction(
                        moveToDropSample3,
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                if (skipDrop && intake.isBreakBeam()) {
                                    skipDrop = false;
//                                    throw new RuntimeException("its running");
                                }

                                return false;
                            }
                        }
                ),
                new DropBlock(),
                moveToGrabSpecimen4,
                moveToScoreSpecimen4,
                new InstantAction(() -> {
                    outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);
                    outtake.setSpecimenDropBehind(false);
                }),
                moveToGrabSpecimen5,
                moveToScoreSpecimen5,
                new InstantAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.OPEN)),
                new Action() {
                    final Action action = new SequentialAction(
                            moveToGrabSpecimen6,
                            moveToScoreSpecimen6);
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!intake.grabbedYellow && !skipDrop) {
                            return action.run(telemetryPacket);
                        } else {
                            return false;
                        }
                    }
                },
                new InstantAction(() -> {
                    outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);
                    outtake.outtakeState(NewOuttake.OuttakeState.IDLE);
                }),
                park,
                new IntakeBlock()
        ));

        while ( !isStopRequested()) {

            masterThread.unThreadedUpdate();

            skipDropTelem.setValue(skipDrop);

            loopTimeTelem.setValue(loopTimer.milliSeconds());

            loopTime = loopTimer.seconds();
            loopTimer.reset();
        }
    }

    public class IntakeBlock implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (intake.getPrevIntakingState() != NewIntake.IntakingState.INTAKING && intake.getPrevIntakingState() != NewIntake.IntakingState.START_INTAKING && intake.getPrevIntakingState() != NewIntake.IntakingState.SERVO_STALL_START_UNJAMMING && intake.getPrevIntakingState() != NewIntake.IntakingState.SERVO_STALL_UNJAMMING_SPIN_OUT) {
                intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                intake.setTargetSlidePos(18.5);
                return false;
            } else {
                extensionDistance = MathUtil.clip(extensionDistance + 20 * loopTime, -.5, 18.5);
                intake.setTargetSlidePos(extensionDistance);
                return true;
            }
        }
    }

    public class RunToTimeThreshold implements Action {
        Action action;

        public RunToTimeThreshold(Action action) {
            this.action = action;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            boolean time = ((30-totalAutoTimer.seconds())>timeThreshold);

            if (!time) {
                skipDrop = true;
                intake.toIntakeState(NewIntake.ToIntakeState.RETRACT);
            }

            return action.run(telemetryPacket) && time ;
        }
    }

    public class DropBlock implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (outtake.getOuttakeState() == NewOuttake.OuttakeState.WAITING_GRAB_SPECIMEN) {
                return false;
            } else if (outtake.getOuttakeState() == NewOuttake.OuttakeState.WAITING_DROP_SAMPLE) {// || intake.getPrevIntakingState() == NewIntake.IntakingState.INTAKING
                outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);
                return false;
            } else {
                return true;
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
//                timeThreshold = -1;
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
                            searchTurn = drivetrain.drive.actionBuilder(MathUtil.toRoadRunnerPose(holdPoint.toPose(Math.toRadians(270))))
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

                        extensionDistance = Math.max(vision.getSampleRobotDiff().getMagnitude() - 9.59029 - intake.getIntakeHorizontalOffset() - intake.getActualSlidePos() - 5.3, 5);//Math.max(extensionDistance+(vision.getSampleRobotDiff().getMagnitude() - 9.59029 - intake.getIntakeHorizontalOffset() - 4)*.125, 3);
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


                    if (drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(1, 1, Math.toRadians(3.5))) && intake.prevSlideError <1) {
                        intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                        intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.INTAKING_1;
                        autoTimer.reset();
                    }
                    break;
                case INTAKING_1:
//hello brett king
                    if (autoTimer.seconds()>.5) {
                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.RETRACTING;
                    } else {
                        targetHeading = MathUtil.clip(Rotation.inRange(prevHeading + Rotation.inRange((vision.getTargetRobotPose().getHeading() - prevHeading), Math.PI, -Math.PI) * .15, 2 * Math.PI, 0), minGrabAngle, maxGrabAngle);

                        drivetrain.holdPoint(holdPoint.toPose(targetHeading));

                        extensionDistance = MathUtil.clip(extensionDistance + 12 * loopTime, -.5, 18.5);
                        intake.setTargetSlidePos(extensionDistance);

                    }
                    break;
                case RETRACTING:

                    if (autoTimer.seconds()>.5+.4 || extensionDistance == 2) {
                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.INTAKING_2;
                    } else {
                        targetHeading = MathUtil.clip(Rotation.inRange(prevHeading + Rotation.inRange((vision.getTargetRobotPose().getHeading() - prevHeading), Math.PI, -Math.PI) * .15, 2 * Math.PI, 0), minGrabAngle, maxGrabAngle);

                        drivetrain.holdPoint(holdPoint.toPose(targetHeading));

                        extensionDistance = MathUtil.clip(extensionDistance - 12 * loopTime, 5, 18.5);
                        intake.setTargetSlidePos(extensionDistance);

                    }
                    break;
                case INTAKING_2:

                    targetHeading = MathUtil.clip(Rotation.inRange(prevHeading + Rotation.inRange((vision.getTargetRobotPose().getHeading() - prevHeading), Math.PI, -Math.PI) * .15, 2 * Math.PI, 0), minGrabAngle, maxGrabAngle);

                    drivetrain.holdPoint(holdPoint.toPose(targetHeading));

                    if (autoTimer.seconds() > 2) {
                        intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION);
                        double curHeading = drivetrain.getPoseEstimate().getHeading();

                        if (curHeading > Math.toRadians(273)) {
                            targetHeading = Math.toRadians(267);
                            drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(265)));
                        } else if (curHeading < Math.toRadians(267)) {
                            targetHeading = Math.toRadians(273);
                            drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(275)));
                        } else {
                            targetHeading = Math.toRadians(265);
                            drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(265)));
                        }

                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.RESETTING;
                    } else {
                        extensionDistance = MathUtil.clip(extensionDistance + 12 * loopTime, -.5, 18.5);
                        intake.setTargetSlidePos(extensionDistance);
                    }

                    break;
                case EJECTING:
                    if (intake.getPrevIntakingState() == NewIntake.IntakingState.IDLE) {

                        intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION);
                        extensionDistance = 1;
                        drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(270)));

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
