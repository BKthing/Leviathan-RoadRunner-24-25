package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive3.PARAMS;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
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
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.NewDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.NewIntake;
import org.firstinspires.ftc.teamcode.subsystems.NewOuttake;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

import java.util.Arrays;

@Autonomous
public class RRRight5plus0Auto extends LinearOpMode {
    NewDrivetrain drivetrain;
    NewIntake intake;
    NewOuttake outtake;
    MasterThread masterThread;
    Telemetry.Item loopTime;

    Boolean blueAlliance = null;

    private Encoder verticalSlideEncoder, horizontalSlideEncoder;

    private TouchSensor breakBeam;


    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTimer loopTimer = new ElapsedTimer();
        loopTime = telemetry.addData("Loop time:", loopTimer.milliSeconds());

        horizontalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalLeft"));
        breakBeam = hardwareMap.get(TouchSensor.class, "breakBeam");

        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);

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





        Action preload = drivetrain.drive.actionBuilder(new Pose2d(-5.5, 62.1, Math.toRadians(270)))
                .setTangent(Math.toRadians(270))
                .afterTime(.3, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
                })
                .splineToConstantHeading(new Vector2d(-5.5, 29), Math.toRadians(270), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-32, PARAMS.maxProfileAccel))
                .build();

        Action moveToGrabBlock1 = drivetrain.drive.actionBuilder(new Pose2d(-5.5, 28.5, Math.toRadians(270)))
                .setTangent(Math.toRadians(90))
                .afterDisp(20, () -> {
                    intake.setTargetSlidePos(18.5);
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                })
                .splineToConstantHeading(new Vector2d(-10, 37), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-25.5, 40.5, Math.toRadians(235)), Math.toRadians(180))
                .turn(Math.toRadians(-75), new TurnConstraints(5 *Math.PI, -2 *Math.PI, 4 *Math.PI))
                .afterTime(.1, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
                    intake.setTargetSlidePos(14);
                })

                .setTangent(new com.reefsharklibrary.data.Vector2d(-35.5, 43).minus(new com.reefsharklibrary.data.Vector2d(-25.5, 40.5)).getDirection())
                .lineToXSplineHeading(-36, Math.toRadians(232))
                .build();

        Action moveToPlaceBlock2 = drivetrain.drive.actionBuilder(new Pose2d(-36, 43, Math.toRadians(232)))
                .afterTime(0, () -> {
                    intake.setTargetSlidePos(18.5);
                })
                .waitSeconds(.3)
                .turn(Math.toRadians(-60), new TurnConstraints(5 *Math.PI, -2 *Math.PI, 4 *Math.PI))
                .afterTime(.1, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.RETRACT);
                })
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-57.5, 12.5, Math.toRadians(270)), Math.toRadians(180), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-25, PARAMS.maxProfileAccel))
                .splineToConstantHeading(new Vector2d(-62.5, 15.5), Math.toRadians(90), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(30))), new ProfileAccelConstraint(-20, PARAMS.maxProfileAccel))
                .lineToYConstantHeading(54, new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-40, PARAMS.maxProfileAccel))
                .splineToConstantHeading(new Vector2d(-58,61.7), Math.toRadians(80), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-25, PARAMS.maxProfileAccel))
                .build();

        MinVelConstraint moveToScoreVelConstraint = new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(80)));
        ProfileAccelConstraint moveToScoreAccelConstraint = new ProfileAccelConstraint(-55, 70);

        Action moveToScoreSpecimen2 = drivetrain.drive.actionBuilder(new Pose2d(-58, 61.7, Math.toRadians(270)))
                .waitSeconds(.3)
                .setTangent(Math.toRadians(305))
                .splineToConstantHeading(new Vector2d(-3.5, 29), Math.toRadians(305), moveToScoreVelConstraint, moveToScoreAccelConstraint)

                .build();

        Action moveToGrabSpecimen3 = drivetrain.drive.actionBuilder(new Pose2d(-3.5, 31, Math.toRadians(270)))
//                .setTangent(Math.toRadians(90))
//                .setTangent(new com.reefsharklibrary.data.Vector2d(-37, 61.7).minus(new com.reefsharklibrary.data.Vector2d(-3.5, 31)).getDirection())
                .afterTime(.75, () -> {
                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.WAIT_DROP_BEHIND);
                })
                .setTangent(Math.toRadians(115))
//                .lineToY(61.7, new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-35, PARAMS.maxProfileAccel))
                .splineToConstantHeading(new Vector2d(-37, 59), Math.toRadians(90))
                .afterTime(.1, () -> {
                    outtake.toClawPosition(NewOuttake.ClawPosition.CLOSED);
                })
                .lineToY(60.9, new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-35, PARAMS.maxProfileAccel))
                .waitSeconds(.1)
                .build();

        Action moveToScoreSpecimen3 = drivetrain.drive.actionBuilder(new Pose2d(-37, 61.7, Math.toRadians(270)))
                .waitSeconds(.3)
                .setTangent(new com.reefsharklibrary.data.Vector2d(-4, 29).minus(new com.reefsharklibrary.data.Vector2d(-37, 61.7)).getDirection())
                .lineToY(29, moveToScoreVelConstraint, moveToScoreAccelConstraint)
//                .setTangent(Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(-4, 29), Math.toRadians(305), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-50, PARAMS.maxProfileAccel))
                .build();

        Action moveToGrabSpecimen4 = drivetrain.drive.actionBuilder(new Pose2d(-4, 29, Math.toRadians(270)))
                .setTangent(Math.toRadians(115))
                .afterTime(.75, () -> {
                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.WAIT_DROP_BEHIND);
                })
                .splineToConstantHeading(new Vector2d(-37, 59), Math.toRadians(90))
                .afterTime(.1, () -> {
                    outtake.toClawPosition(NewOuttake.ClawPosition.CLOSED);
                })
                .lineToY(60.9, new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-35, PARAMS.maxProfileAccel))
                .waitSeconds(.1)
                .build();

        Action moveToScoreSpecimen4 = drivetrain.drive.actionBuilder(new Pose2d(-37, 60.7, Math.toRadians(270)))
                .waitSeconds(.3)
                .setTangent(new com.reefsharklibrary.data.Vector2d(-4, 29).minus(new com.reefsharklibrary.data.Vector2d(-37, 60.7)).getDirection())
                .lineToY(29, moveToScoreVelConstraint, moveToScoreAccelConstraint)
                .build();

        Action moveToGrabSpecimen5 = drivetrain.drive.actionBuilder(new Pose2d(-4, 29, Math.toRadians(270)))
                .setTangent(Math.toRadians(115))
                .afterTime(.75, () -> {
                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.WAIT_DROP_BEHIND);
                })
                .splineToConstantHeading(new Vector2d(-37, 59), Math.toRadians(90))
                .afterTime(.1, () -> {
                    outtake.toClawPosition(NewOuttake.ClawPosition.CLOSED);
                })
                .lineToY(60.9, new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel))), new ProfileAccelConstraint(-35, PARAMS.maxProfileAccel))
                .waitSeconds(.1)
                .build();

        Action moveToScoreSpecimen5 = drivetrain.drive.actionBuilder(new Pose2d(-37, 60.7, Math.toRadians(270)))
                .waitSeconds(.3)
                .setTangent(new com.reefsharklibrary.data.Vector2d(-4, 29).minus(new com.reefsharklibrary.data.Vector2d(-37, 60.7)).getDirection())
                .lineToY(29, moveToScoreVelConstraint, moveToScoreAccelConstraint)
                .build();

        Action moveToPark = drivetrain.drive.actionBuilder(new Pose2d(-4, 28, Math.toRadians(270)))
                .afterTime(.4, () -> {
                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.RETRACT_FROM_PLACE_BEHIND);
                })
                .setTangent(Math.toRadians(90))
                .afterTime(.6, () -> {
                    intake.setTargetSlidePos(18.5);
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                })
                .splineToSplineHeading(new Pose2d(-22, 47, Math.toRadians(155)), Math.toRadians(165), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(80))), new ProfileAccelConstraint(-35, 75))
                .build();

        drivetrain.drive.setPoseEstimate(new Pose2d(1, 1, 0));
        drivetrain.drive.pinpoint.update();

        if (drivetrain.drive.pinpoint.fastIsPinpointCooked()) {
            throw new RuntimeException("pinpoint cooked");
        }

        waitForStart();

        drivetrain.drive.setPoseEstimate(new  Pose2d(-5.5, 62.1, Math.toRadians(270)));

        masterThread.clearBulkCache();

        intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
        outtake.toOuttakeState(NewOuttake.ToOuttakeState.PLACE_FRONT);

        drivetrain.followPath(new SequentialAction(
                preload,
                new InstantAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.OPEN)),
                moveToGrabBlock1,
                new InstantAction(() -> intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE)),
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
                new InstantAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.OPEN)),
                moveToPark
                ));

        while ( !isStopRequested()) {

            masterThread.unThreadedUpdate();

            loopTime.setValue(loopTimer.milliSeconds());

            loopTimer.reset();
        }
    }

}
