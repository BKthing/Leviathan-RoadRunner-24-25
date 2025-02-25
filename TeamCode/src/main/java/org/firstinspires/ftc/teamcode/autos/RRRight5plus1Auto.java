package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive3.PARAMS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
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
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.NewDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.NewIntake;
import org.firstinspires.ftc.teamcode.subsystems.NewOuttake;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

import java.util.Arrays;

@Autonomous
public class RRRight5plus1Auto extends LinearOpMode {
    NewDrivetrain drivetrain;
    NewIntake intake;
    NewOuttake outtake;
    MasterThread masterThread;
    Telemetry.Item loopTimeTelem;

    Boolean blueAlliance = null;

    ElapsedTimer autoTimer = new ElapsedTimer();

    double loopTime = .05;

    double extensionDistance = 0;

    private Encoder verticalSlideEncoder, horizontalSlideEncoder;

    private TouchSensor breakBeam;


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


        //its important that outtake is added after intake for update order purposes
        masterThread.addSubSystems(
                drivetrain,
                intake,
                outtake
        );





        Action preload = drivetrain.drive.actionBuilder(new Pose2d(-5.5, 62.1, Math.toRadians(270)))
                .setTangent(Math.toRadians(270))
                .afterTime(.3, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
                })
                .splineToConstantHeading(new Vector2d(-5.5, 29.5), Math.toRadians(270), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-45, 50))
                .build();

        Action moveToGrabBlock1 = drivetrain.drive.actionBuilder(new Pose2d(-5.5, 30, Math.toRadians(270)))
                .setTangent(Math.toRadians(90))
                .afterDisp(20, () -> {
                    intake.setTargetSlidePos(18.5);
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE_AUTO_SHOVE_HEIGHT);
                })
                .splineToConstantHeading(new Vector2d(-10, 37), Math.toRadians(180))
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
                .splineToConstantHeading(new Vector2d(-60.5, 15.5), Math.toRadians(90), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(35))), new ProfileAccelConstraint(-20, PARAMS.maxProfileAccel))
                .lineToYConstantHeading(51.5, new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-40, PARAMS.maxProfileAccel))
                .splineToConstantHeading(new Vector2d(-55.5,60.7), Math.toRadians(80), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-35, PARAMS.maxProfileAccel))
                .build();

        MinVelConstraint moveToScoreVelConstraint = new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(120)));
        ProfileAccelConstraint moveToScoreAccelConstraint = new ProfileAccelConstraint(-65, 80);

        MinVelConstraint firstMoveToGrabVelConstraint = new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(70)));
        ProfileAccelConstraint firstMoveToGrabAccelConstraint = new ProfileAccelConstraint(-30, 60);

        MinVelConstraint moveToGrabVelConstraint = new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(70)));
        ProfileAccelConstraint moveToGrabAccelConstraint = new ProfileAccelConstraint(-35, 60);

        Action moveToScoreSpecimen2 = drivetrain.drive.actionBuilder(new Pose2d(-55.5, 60.7, Math.toRadians(270)))
                .waitSeconds(.2)
                .setTangent(Math.toRadians(305))
                .splineToConstantHeading(new Vector2d(-3.5, 29.5), Math.toRadians(300), moveToScoreVelConstraint, moveToScoreAccelConstraint)

                .build();

        Action moveToGrabSpecimen3 = drivetrain.drive.actionBuilder(new Pose2d(-3.5, 29.5, Math.toRadians(270)))
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

        Action moveToScoreSpecimen3 = drivetrain.drive.actionBuilder(new Pose2d(-37, 60.3, Math.toRadians(270)))
                .setTangent(new com.reefsharklibrary.data.Vector2d(-4, 29.5).minus(new com.reefsharklibrary.data.Vector2d(-37, 61.7)).getDirection())
                .lineToY(29.5, moveToScoreVelConstraint, moveToScoreAccelConstraint)
//                .setTangent(Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(-4, 29), Math.toRadians(305), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-50, PARAMS.maxProfileAccel))
                .build();

        Action moveToGrabSpecimen4 = drivetrain.drive.actionBuilder(new Pose2d(-4, 29.5, Math.toRadians(270)))
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

        Action moveToScoreSpecimen4 = drivetrain.drive.actionBuilder(new Pose2d(-37, 60.3, Math.toRadians(270)))
                .setTangent(new com.reefsharklibrary.data.Vector2d(-4, 29.5).minus(new com.reefsharklibrary.data.Vector2d(-37, 60.7)).getDirection())
                .lineToY(29.5, moveToScoreVelConstraint, moveToScoreAccelConstraint)
                .build();

        Action moveToGrabSpecimen5 = drivetrain.drive.actionBuilder(new Pose2d(-4, 29.5, Math.toRadians(270)))
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

        Action moveToScoreSpecimen5 = drivetrain.drive.actionBuilder(new Pose2d(-37, 60.3, Math.toRadians(270)))
                .setTangent(new com.reefsharklibrary.data.Vector2d(-4, 29.5).minus(new com.reefsharklibrary.data.Vector2d(-37, 60.7)).getDirection())
                .lineToY(29.5, moveToScoreVelConstraint, moveToScoreAccelConstraint)
                .build();

        Action moveToGrab3 = drivetrain.drive.actionBuilder(new Pose2d(-4, 29.5, Math.toRadians(270)))
                .afterTime(.6, () -> {
                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.RETRACT_FROM_PLACE_BEHIND);
                })
                .afterTime(1.2, () -> {
                    intake.setTargetSlidePos(15);
                    extensionDistance = 15;
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                    autoTimer.reset();
                })
                .splineToLinearHeading(new Pose2d(-11.5, 56.5, Math.toRadians(175)), Math.toRadians(90))
                .build();

        Action scoreBasket = drivetrain.drive.actionBuilder(new Pose2d(-11.5, 56, Math.toRadians(180)))
                .lineToXLinearHeading(55.5, Math.toRadians(193), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(80))), new ProfileAccelConstraint(-35, 65))
                .build();

        Action moveToPark = drivetrain.drive.actionBuilder(new Pose2d(55.5, 56, Math.toRadians(189)))
                .setTangent(Math.toRadians(180))
                .afterTime(.4, () -> {
                    intake.setTargetSlidePos(18.5);
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                })
                .lineToXConstantHeading(-14, new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(80))), new ProfileAccelConstraint(-60, 75))
                .build();

        drivetrain.drive.setPoseEstimate(new Pose2d(1, 1, 0));
        drivetrain.drive.pinpoint.update();

        if (drivetrain.drive.pinpoint.fastIsPinpointCooked()) {
            throw new RuntimeException("pinpoint cooked");
        }

        Action intakeBlock = new IntakeBlock();

        waitForStart();

        drivetrain.drive.setPoseEstimate(new  Pose2d(-5.5, 62.1, Math.toRadians(270)));

        masterThread.clearBulkCache();

        intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
        outtake.toOuttakeState(NewOuttake.ToOuttakeState.FIRST_PLACE_FRONT);

        drivetrain.followPath(new SequentialAction(
                preload,
                new InstantAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.OPEN)),
                moveToGrabBlock1,
                new InstantAction(() -> intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE_AUTO_SHOVE_HEIGHT)),
                moveToPlaceBlock2,
                new InstantAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.CLOSED)),
                moveToScoreSpecimen2,
                new InstantAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.OPEN)),
                moveToGrabSpecimen3,
//                new InstantAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.CLOSED)),
                moveToScoreSpecimen3,
                new InstantAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.OPEN)),
                moveToGrabSpecimen4,
//                new InstantAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.CLOSED)),
                moveToScoreSpecimen4,
                new InstantAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.OPEN)),
                moveToGrabSpecimen5,
//                new InstantAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.CLOSED)),
                moveToScoreSpecimen5,
                new InstantAction(() -> {
                    outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);
                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.IDLE);
                }),
                moveToGrab3,
                intakeBlock,
                scoreBasket,
                new ScoreBlock(),
                new SleepAction(.2),
                new InstantAction(() -> {
                    drivetrain.holdPoint(new com.reefsharklibrary.data.Pose2d(-14, 56, Math.toRadians(180)));
                }),
                new SleepAction(.3),
                new InstantAction(() -> {
                    intake.setTargetSlidePos(18.5);
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                })
//                moveToPark
                ));

        while ( !isStopRequested()) {

            masterThread.unThreadedUpdate();

            loopTimeTelem.setValue(loopTimer.milliSeconds());

            loopTime = loopTimer.seconds();
            loopTimer.reset();
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
//                if (!intake.isBreakBeam() && (intake.getPrevIntakeState() == NewIntake.IntakeState.RETRACTING_INTAKE || intake.getPrevIntakeState() == NewIntake.IntakeState.RETRACTING || intake.getPrevIntakeState() == NewIntake.IntakeState.WAITING_AFTER_RETRACTING || intake.getPrevIntakeState() == NewIntake.IntakeState.WAITING_FOR_TRANSFER)) {
//                    intake.setIntakingState(NewIntake.IntakingState.START_EJECTING_PARTIAL_GRAB);
//                    return false;
//                } else
                {
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

}
