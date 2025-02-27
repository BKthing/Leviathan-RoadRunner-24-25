package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive3.PARAMS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
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
public class RRTest extends LinearOpMode {

    public enum GrabFromSubmersibleState {
        SEARCHING,
        APPROACHING,
        INTAKING,
        EJECTING,
        RESETTING
    }

    private double targetHeading = Math.toRadians(180);
    private double prevHeading = Math.toRadians(180);

    private final double maxGrabAngle = Math.toRadians(190);

    NewDrivetrain drivetrain;
    NewIntake intake;
    NewOuttake outtake;
    VisionSubsystem vision;
    MasterThread masterThread;
    Telemetry.Item loopTime;

    Boolean blueAlliance = null;

    private Encoder verticalSlideEncoder, horizontalSlideEncoder;

    private TouchSensor breakBeam;

    double extensionDistance = 0;

    private final com.reefsharklibrary.data.Vector2d holdPoint = new com.reefsharklibrary.data.Vector2d(22, 10);

    private final ElapsedTimer loopTimer = new ElapsedTimer();

    private final ElapsedTimer autoTimer = new ElapsedTimer();

    private final ElapsedTimer totalAutoTimer = new ElapsedTimer();

    double timeThreshold = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        loopTime = telemetry.addData("Loop time:", loopTimer.milliSeconds());

        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);

        horizontalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalLeft"));
        breakBeam = hardwareMap.get(TouchSensor.class, "breakBeam");

        verticalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalLeft"));

        intake = new NewIntake(masterThread.getData(), horizontalSlideEncoder, breakBeam, blueAlliance, false, true, () -> drivetrain.getVoltage());

        outtake = new NewOuttake(masterThread.getData(), intake, verticalSlideEncoder, blueAlliance, false, true, true, true, () -> drivetrain.getVoltage());

        drivetrain = new NewDrivetrain(masterThread.getData(), outtake, intake);
        drivetrain.setDriveState(NewDrivetrain.DriveState.FOLLOW_PATH);



        vision = new VisionSubsystem(drivetrain, masterThread.getData(), blueAlliance, false);

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
                .afterTime(.6, () -> {//0
                    intake.setTargetSlidePos(14);
                    extensionDistance = 14;
                })
                .splineToLinearHeading(new Pose2d(62, 54, Math.toRadians(250)), Math.toRadians(0))
                .build();

        Action moveToGrabBlock1 = drivetrain.drive.actionBuilder(new Pose2d(61, 54, Math.toRadians(250)))
                .setTangent(Math.toRadians(250))
                .afterTime(0, () -> {
//                    intake.setTargetSlidePos(9);
//                    extensionDistance = 9;
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                    autoTimer.reset();
                })
                .splineToConstantHeading(new Vector2d(56, 51), Math.toRadians(250))
//                .splineToLinearHeading(new Pose2d(50.5, 50, Math.toRadians(270)), Math.toRadians(270))
                .build();

        Action moveToScoreBlock1 = drivetrain.drive.actionBuilder(new Pose2d(56, 51, Math.toRadians(270)))
                .setTangent(Math.toRadians(84))
                .splineToLinearHeading(new Pose2d(62, 53, Math.toRadians(265)), Math.toRadians(85))
                .build();

        Action moveToGrabBlock2 = drivetrain.drive.actionBuilder(new Pose2d(62, 53, Math.toRadians(265)))
                .afterTime(0, () -> {
                    intake.setTargetSlidePos(10.5);
                    extensionDistance = 10.5;
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
                .splineToLinearHeading(new Pose2d(62.5, 54, Math.toRadians(270)), Math.toRadians(90))
                .build();

        Action moveToGrabBlock3 = drivetrain.drive.actionBuilder(new Pose2d(62.5, 54, Math.toRadians(270)))
                .afterTime(0, () -> {
                    intake.setTargetSlidePos(12.5);
                    extensionDistance = 12.5;
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                    autoTimer.reset();
                })
                .waitSeconds(.1)
                .setTangent(270)
                .splineToLinearHeading(new Pose2d(62.5, 51.5, Math.toRadians(278)), Math.toRadians(270))
                .build();

        Action moveToScoreBlock3 = drivetrain.drive.actionBuilder(new Pose2d(62.5, 51.5, Math.toRadians(278)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(62.5, 55, Math.toRadians(270)), Math.toRadians(90))
                .build();

        Action moveToSubmersible1 = new Action() {
            boolean notCanceled = true;

            final Action action = drivetrain.drive.actionBuilder(new Pose2d(62.5, 55, Math.toRadians(240)))
                    .setTangent(Math.toRadians(235))
                    .afterDisp(37, () -> {
                        intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION);
                        drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(180)));
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

            final Action action = drivetrain.drive.actionBuilder(new Pose2d(54, 54, Math.toRadians(240)))
                    .setTangent(Math.toRadians(235))
                    .afterDisp(37, () -> {
                        intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION);
                        drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(180)));
                        notCanceled = false;
                    })
                    .lineToYLinearHeading(0, Math.toRadians(190), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-60, PARAMS.maxProfileAccel))
                    .build();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return notCanceled && action.run(telemetryPacket);
            }
        };


        Action scoreBasket = drivetrain.drive.actionBuilder(new Pose2d(-10, 56, Math.toRadians(180)))
                .lineToXLinearHeading(55, Math.toRadians(189), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(80))), new ProfileAccelConstraint(-40, 75))
                .build();

        Action moveToPark = drivetrain.drive.actionBuilder(new Pose2d(10, 56, Math.toRadians(180)))
                .setTangent(Math.toRadians(180))
                .afterTime(.6, () -> {
                    intake.setTargetSlidePos(18.5);
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                })
                .lineToXConstantHeading(14, new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(80))), new ProfileAccelConstraint(-60, 75))
                .build();


        Action park = new Action() {
            boolean notCanceled = true;

            final Action action = drivetrain.drive.actionBuilder(new Pose2d(54, 54, Math.toRadians(240)))
                    .setTangent(Math.toRadians(235))
                    .afterTime(0, () -> {
                        drivetrain.cancelHoldPoint();
                    })
                    .afterDisp(30, () -> {
                        intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
                        outtake.toOuttakeState(NewOuttake.ToOuttakeState.TOUCH_BAR);
                    })
                    .afterDisp(37, () -> {
                        drivetrain.holdPoint(new com.reefsharklibrary.data.Pose2d(20, 10, Math.toRadians(180)));
                        notCanceled = false;
                    })
                    .lineToYLinearHeading(0, Math.toRadians(190), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-60, PARAMS.maxProfileAccel))
                    .build();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return notCanceled && action.run(telemetryPacket);
            }
        };

        drivetrain.drive.setPoseEstimate(new Pose2d(1, 1, 0));
        drivetrain.drive.pinpoint.update();

        if (drivetrain.drive.pinpoint.fastIsPinpointCooked()) {
            throw new RuntimeException("pinpoint cooked");
        }

        waitForStart();

        totalAutoTimer.reset();

        drivetrain.drive.setPoseEstimate(new Pose2d(-10, 56, Math.toRadians(180)));

        outtake.toOuttakeState(NewOuttake.ToOuttakeState.RETRACT_FROM_PLACE_BEHIND);

        masterThread.clearBulkCache();

//        intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
//        outtake.toOuttakeState(NewOuttake.ToOuttakeState.PLACE_BEHIND);

        drivetrain.followPath(new SequentialAction(
                new SequentialAction(
                    scoreBasket,
                    moveToPark
                )

        ));

        while ( !isStopRequested()) {

            masterThread.unThreadedUpdate();

            loopTime.setValue(loopTimer.milliSeconds());

            loopTimer.reset();
        }
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
                extensionDistance = MathUtil.clip(extensionDistance + 20 * loopTimer.seconds(), -.5, 18.5);
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
                if (intake.getPrevIntakingState() == NewIntake.IntakingState.INTAKING) {
                    intake.setIntakingState(NewIntake.IntakingState.START_EJECTING_PARTIAL_GRAB);
                    return false;
                } else {
                    firstLoop = false;
                    autoTimer.reset();
                    return true;
                }
            } else {
                if (outtake.getOuttakeState() == NewOuttake.OuttakeState.WAITING_PLACE_BEHIND || outtake.getFailedToTransfer() || intake.getPrevIntakingState() == NewIntake.IntakingState.INTAKING) {
                    outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);
                    return false;
                } else {
                    return true;
                }
            }
        }
    }

    public class GrabFromSubmersible implements Action {

        RRTest.GrabFromSubmersibleState grabFromSubmersibleState = RRTest.GrabFromSubmersibleState.SEARCHING;


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (intake.getPrevIntakingState() == NewIntake.IntakingState.FINISH_EJECTING || intake.getPrevIntakingState() == NewIntake.IntakingState.START_EJECTING) {

                grabFromSubmersibleState = RRTest.GrabFromSubmersibleState.EJECTING;
            }

            switch (grabFromSubmersibleState) {
                case SEARCHING:
                    if (vision.hasSample()) {
                        targetHeading = prevHeading+Rotation.inRange((vision.getTargetRobotPose().getHeading()-prevHeading), Math.PI, -Math.PI)*.3;

                        drivetrain.holdPoint(holdPoint.toPose(targetHeading));

                        extensionDistance = Math.max(vision.getSampleRobotDiff().getMagnitude() - 9.59029 - intake.getIntakeHorizontalOffset() - intake.getActualSlidePos() - 4.7, 0);//Math.max(extensionDistance+(vision.getSampleRobotDiff().getMagnitude() - 9.59029 - intake.getIntakeHorizontalOffset() - 4)*.125, 3);
                        intake.setTargetSlidePos(extensionDistance);

                        autoTimer.reset();
                        grabFromSubmersibleState = RRTest.GrabFromSubmersibleState.APPROACHING;
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
                    targetHeading = prevHeading+Rotation.inRange((vision.getTargetRobotPose().getHeading()-prevHeading), Math.PI, -Math.PI)*.15;

                    drivetrain.holdPoint(holdPoint.toPose(targetHeading));//, maxGrabAngle


                    if (drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(1, 1, Math.toRadians(3.5)))) {
                        intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                        intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                        grabFromSubmersibleState = RRTest.GrabFromSubmersibleState.INTAKING;
                    }
                    break;
                case INTAKING:
                    if (intake.getPrevIntakingState() != NewIntake.IntakingState.INTAKING && intake.getPrevIntakingState() != NewIntake.IntakingState.START_INTAKING && intake.getPrevIntakingState() != NewIntake.IntakingState.SERVO_STALL_START_UNJAMMING && intake.getPrevIntakingState() != NewIntake.IntakingState.SERVO_STALL_UNJAMMING_SPIN_OUT) {
                        return false;
                    }

                    targetHeading = prevHeading+Rotation.inRange((vision.getTargetRobotPose().getHeading()-prevHeading), Math.PI, -Math.PI)*.15;

                    drivetrain.holdPoint(holdPoint.toPose(targetHeading));

                    if (autoTimer.seconds() > 5) {
                        intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION);
                        double curHeading = drivetrain.getPoseEstimate().getHeading();

                        if (curHeading > Math.toRadians(185)) {
                            targetHeading = Math.toRadians(175);
                            drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(175)));
                        } else if (curHeading < Math.toRadians(175)) {
                            targetHeading = Math.toRadians(185);
                            drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(185)));
                        } else {
                            targetHeading = Math.toRadians(170);
                            drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(170)));
                        }

                        grabFromSubmersibleState = RRTest.GrabFromSubmersibleState.RESETTING;
                    } else {
                        extensionDistance = MathUtil.clip(extensionDistance + 5 * loopTimer.seconds(), -.5, 18.5);
                        intake.setTargetSlidePos(extensionDistance);
                    }
                    break;
                case EJECTING:
                    if (intake.getPrevIntakingState() == NewIntake.IntakingState.IDLE) {

                        intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION);
                        extensionDistance = 1;
                        drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(180)));

                        grabFromSubmersibleState = RRTest.GrabFromSubmersibleState.RESETTING;
                    }
                    break;
                case RESETTING:
                    if (drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(1, 1, Math.toRadians(3.5)))) {
                        grabFromSubmersibleState = RRTest.GrabFromSubmersibleState.SEARCHING;
                        extensionDistance = 1;
                    }
                    break;
            }

            prevHeading = targetHeading;

            return true;


        }
    }

}
