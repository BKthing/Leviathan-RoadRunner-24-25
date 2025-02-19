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
public class RRLeft1plus3Auto extends LinearOpMode {
    NewDrivetrain drivetrain;
    NewIntake intake;
    NewOuttake outtake;
    MasterThread masterThread;
    Telemetry.Item loopTime;

    Boolean blueAlliance = null;

    private Encoder verticalSlideEncoder, horizontalSlideEncoder;

    private TouchSensor breakBeam;

    double extensionDistance = 0;

    private final ElapsedTimer loopTimer = new ElapsedTimer();

    private final ElapsedTimer autoTimer = new ElapsedTimer();

    @Override
    public void runOpMode() throws InterruptedException {
        loopTime = telemetry.addData("Loop time:", loopTimer.milliSeconds());

        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);

        horizontalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalLeft"));
        breakBeam = hardwareMap.get(TouchSensor.class, "breakBeam");

        intake = new NewIntake(masterThread.getData(), horizontalSlideEncoder, breakBeam, blueAlliance, false, true, () -> drivetrain.getVoltage());

        drivetrain = new NewDrivetrain(masterThread.getData(), intake);
        drivetrain.setDriveState(NewDrivetrain.DriveState.FOLLOW_PATH);





        verticalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalLeft"));

        outtake = new NewOuttake(masterThread.getData(), intake, verticalSlideEncoder, blueAlliance, false, true, true, true, () -> drivetrain.getVoltage());

        //its important that outtake is added after intake for update order purposes
        masterThread.addSubSystems(
                drivetrain,
                intake,
                outtake
        );

        Action intakeBlock = new IntakeBlock();


        Action preload = drivetrain.drive.actionBuilder(new Pose2d(16.8, 62.1, Math.toRadians(270)))
                .setTangent(Math.toRadians(270))
                .afterTime(.3, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
                })
                .splineToConstantHeading(new Vector2d(5.5, 29), Math.toRadians(270))
                .build();

        Action moveToGrabBlock1 = drivetrain.drive.actionBuilder(new Pose2d(5.5, 29, Math.toRadians(270)))
                .setTangent(Math.toRadians(90))
                .afterTime(1.3, () -> {
                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.RETRACT_FROM_PLACE_BEHIND);
                })
                .lineToY(31, new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-50, 80))
                .splineToConstantHeading(new Vector2d(45, 56), Math.toRadians(0))
                .afterTime(0, () -> {
                    intake.setTargetSlidePos(9);
                    extensionDistance = 9;
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                    autoTimer.reset();
                })
                .splineToConstantHeading(new Vector2d(49.5, 50), Math.toRadians(270))
                .build();

        Action moveToScoreBlock1 = drivetrain.drive.actionBuilder(new Pose2d(49.5, 50, Math.toRadians(270)))
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(59.5, 52.5, Math.toRadians(225)), Math.toRadians(45))
                .build();

        Action moveToGrabBlock2 = drivetrain.drive.actionBuilder(new Pose2d(59.5, 52.5, Math.toRadians(225)))
                .waitSeconds(.1)
                .setTangent(270)
                .afterTime(0, () -> {
                    intake.setTargetSlidePos(11);
                    extensionDistance = 11;
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                    autoTimer.reset();
                })
                .splineToLinearHeading(new Pose2d(57.5, 51.5, Math.toRadians(275)), Math.toRadians(270))
                .build();

        Action moveToScoreBlock2 = drivetrain.drive.actionBuilder(new Pose2d(57.5, 51.5, Math.toRadians(275)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(58, 54, Math.toRadians(225)), Math.toRadians(90))
                .build();

        Action moveToGrabBlock3 = drivetrain.drive.actionBuilder(new Pose2d(59, 54, Math.toRadians(225)))
                .waitSeconds(.1)
                .setTangent(270)
                .afterTime(0, () -> {
                    intake.setTargetSlidePos(12);
                    extensionDistance = 12;
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                    autoTimer.reset();
                })
                .splineToLinearHeading(new Pose2d(57.5, 50, Math.toRadians(298)), Math.toRadians(270))
                .build();

        Action moveToScoreBlock3 = drivetrain.drive.actionBuilder(new Pose2d(57.5, 50, Math.toRadians(298)))
                .setTangent(Math.toRadians(105))
                .splineToLinearHeading(new Pose2d(58.5, 53.5, Math.toRadians(225)), Math.toRadians(90))
                .build();

        Action park = drivetrain.drive.actionBuilder(new Pose2d(58.5, 53.5, Math.toRadians(225)))
                .afterTime(.5, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
                })
                .afterTime(1, () -> {
                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.TOUCH_BAR);
                })
                .setTangent(Math.toRadians(240))
                .splineToLinearHeading(new Pose2d(35, 6, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(20, 6, Math.toRadians(180)), Math.toRadians(180))
                .build();

        drivetrain.drive.setPoseEstimate(new Pose2d(1, 1, 0));
        drivetrain.drive.pinpoint.update();

        if (drivetrain.drive.pinpoint.isPinpointCooked()) {
            throw new RuntimeException("pinpoint cooked");
        }

        waitForStart();

        drivetrain.drive.setPoseEstimate(new Pose2d(16.8, 62.1, Math.toRadians(270)));

        masterThread.clearBulkCache();

        intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
        outtake.toOuttakeState(NewOuttake.ToOuttakeState.PLACE_FRONT);

        drivetrain.followPath(new SequentialAction(
                preload,
                new InstantAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.OPEN)),
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
                park
                ));

        while ( !isStopRequested()) {

            masterThread.unThreadedUpdate();

            loopTime.setValue(loopTimer.milliSeconds());

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

}
