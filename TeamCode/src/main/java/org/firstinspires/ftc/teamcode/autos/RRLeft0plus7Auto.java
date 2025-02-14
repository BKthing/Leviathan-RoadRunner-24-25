package org.firstinspires.ftc.teamcode.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
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

@Autonomous
public class RRLeft0plus7Auto extends LinearOpMode {

    public enum GrabFromSubmersibleState {
        SEARCHING,
        APPROACHING,
        INTAKING,
        RESETTING
    }

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

    private final com.reefsharklibrary.data.Vector2d holdPoint = new com.reefsharklibrary.data.Vector2d(22, 6);

    private final ElapsedTimer loopTimer = new ElapsedTimer();

    private final ElapsedTimer autoTimer = new ElapsedTimer();

    private final ElapsedTimer totalAutoTimer = new ElapsedTimer();

    double timeThreshold = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        loopTime = telemetry.addData("Loop time:", loopTimer.milliSeconds());

        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);

        drivetrain = new NewDrivetrain(masterThread.getData(), intake);
        drivetrain.setDriveState(NewDrivetrain.DriveState.FOLLOW_PATH);

        horizontalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalLeft"));
        breakBeam = hardwareMap.get(TouchSensor.class, "breakBeam");

        intake = new NewIntake(masterThread.getData(), horizontalSlideEncoder, breakBeam, blueAlliance, false, true, () -> drivetrain.getVoltage());


        verticalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalLeft"));

        outtake = new NewOuttake(masterThread.getData(), intake, verticalSlideEncoder, blueAlliance, false, true, true, true, () -> drivetrain.getVoltage());

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
                .setTangent(Math.toRadians(300))
                .afterTime(.3, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
                })
                .afterTime(.0, () -> {//0
                    intake.setTargetSlidePos(16);
                    extensionDistance = 16;
                })
                .splineToLinearHeading(new Pose2d(62, 54, Math.toRadians(250)), Math.toRadians(45))
                .build();

        Action moveToGrabBlock1 = drivetrain.drive.actionBuilder(new Pose2d(61, 55, Math.toRadians(250)))
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
                .splineToLinearHeading(new Pose2d(61.5, 51, Math.toRadians(265)), Math.toRadians(85))
                .build();

        Action moveToGrabBlock2 = drivetrain.drive.actionBuilder(new Pose2d(61.5, 51, Math.toRadians(265)))
                .afterTime(0, () -> {
                    intake.setTargetSlidePos(11);
                    extensionDistance = 11;
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
                .splineToLinearHeading(new Pose2d(62.5, 52, Math.toRadians(270)), Math.toRadians(90))
                .build();

        Action moveToGrabBlock3 = drivetrain.drive.actionBuilder(new Pose2d(62.5, 52, Math.toRadians(270)))
                .afterTime(0, () -> {
                    intake.setTargetSlidePos(13);
                    extensionDistance = 13;
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
                .splineToLinearHeading(new Pose2d(62.5, 53, Math.toRadians(270)), Math.toRadians(90))
                .build();

        Action moveToSubmersible = drivetrain.drive.actionBuilder(new Pose2d(55, 55, Math.toRadians(45)))
                .setTangent(Math.toRadians(225))
                .splineTo(new Vector2d(35, 6), Math.toRadians(180))
                .lineToX(22)
                .afterTime(.3, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION);
                })
                .build();

        Action scoreToScoreFromSubmersible = drivetrain.drive.actionBuilder(new Pose2d(22, 6, Math.toRadians(0)))
                .lineToX(35)
                .splineTo(new Vector2d(55, 55), Math.toRadians(45))
                .build();

        Action park = drivetrain.drive.actionBuilder(new Pose2d(55, 55, Math.toRadians(45)))
                .afterTime(.5, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
                })
                .afterTime(1, () -> {
                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.TOUCH_BAR);
                })
                .setTangent(Math.toRadians(225))
                .splineTo(new Vector2d(35, 6), Math.toRadians(180))
                .lineToX(20)
//                .splineToConstantHeading(new Vector2d(20, 6), Math.toRadians(180))
                .build();

        waitForStart();

        totalAutoTimer.reset();

        drivetrain.drive.setPoseEstimate(new Pose2d(38.93, 60.23, Math.toRadians(180)));
        drivetrain.drive.pinpoint.setPosition(new  Pose2d(38.93, 60.23, Math.toRadians(180)));


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
                                moveToSubmersible,
                                new GrabFromSubmersible(),
                                scoreToScoreFromSubmersible,
                                new ScoreBlock()
                        )
                ),
                park
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

        GrabFromSubmersibleState grabFromSubmersibleState = GrabFromSubmersibleState.SEARCHING;


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (intake.getPrevIntakingState() != NewIntake.IntakingState.INTAKING && intake.getPrevIntakingState() != NewIntake.IntakingState.START_INTAKING && intake.getPrevIntakingState() != NewIntake.IntakingState.SERVO_STALL_START_UNJAMMING && intake.getPrevIntakingState() != NewIntake.IntakingState.SERVO_STALL_UNJAMMING_SPIN_OUT) {
                return false;
            } else {
                switch (grabFromSubmersibleState) {
                    case SEARCHING:
                        if (vision.hasSample()) {
                            drivetrain.holdPoint(holdPoint.toPose(vision.getTargetRobotPose().getHeading()));

                            extensionDistance = Math.max(vision.getSampleRobotDiff().getMagnitude()-9.59029-intake.getIntakeHorizontalOffset()-4, 3);
                            intake.setTargetSlidePos(extensionDistance);

                            autoTimer.reset();
                            grabFromSubmersibleState = GrabFromSubmersibleState.APPROACHING;
                        }
                        break;
                    case APPROACHING:
                        //adjustSlides
                        double newExtensionDistance = Math.max(vision.getSampleRobotDiff().getMagnitude()-9.59029-intake.getIntakeHorizontalOffset()-4, 3);
                        if (Math.abs(newExtensionDistance-extensionDistance)>.2) {
                            extensionDistance = newExtensionDistance;
                            intake.setTargetSlidePos(extensionDistance);
                        }

                        //adjust holdPoint heading
                        drivetrain.holdPoint(holdPoint.toPose(Math.min(Rotation.inRange(vision.getTargetRobotPose().getHeading(), 2*Math.PI, 0), maxGrabAngle)));


                        if (drivetrain.getHoldPointError().inRange(new com.reefsharklibrary.data.Pose2d(1, 1, Math.toRadians(2)))) {
                            intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                            intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);                        }
                        break;
                    case INTAKING:
                        drivetrain.holdPoint(holdPoint.toPose(Math.min(Rotation.inRange(vision.getTargetRobotPose().getHeading(), 2*Math.PI, 0), maxGrabAngle)));

                        if (autoTimer.seconds()>4) {
                            intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION);
                            double curHeading = drivetrain.getPoseEstimate().getHeading();

                            if (curHeading>Math.toRadians(185)) {
                                drivetrain.holdPoint(holdPoint.toPose(175));
                            } else if (curHeading<Math.toRadians(175)) {
                                drivetrain.holdPoint(holdPoint.toPose(185));
                            } else {
                                drivetrain.holdPoint(holdPoint.toPose(170));
                            }

                            grabFromSubmersibleState = GrabFromSubmersibleState.RESETTING;
                        } else {
                            extensionDistance = MathUtil.clip(extensionDistance + 20 * loopTimer.seconds(), -.5, 18.5);
                            intake.setTargetSlidePos(extensionDistance);
                        }
                        break;
                    case RESETTING:
                        if (drivetrain.getHoldPointError().inRange(new com.reefsharklibrary.data.Pose2d(1, 1, Math.toRadians(2)))) {
                            grabFromSubmersibleState = GrabFromSubmersibleState.SEARCHING;                       }
                        break;
                }


                return true;
            }


        }
    }

}
