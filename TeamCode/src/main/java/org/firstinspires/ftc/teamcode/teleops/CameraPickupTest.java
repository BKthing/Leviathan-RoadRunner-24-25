package org.firstinspires.ftc.teamcode.teleops;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.reefsharklibrary.data.Rotation;
import com.reefsharklibrary.data.Vector2d;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autos.RRLeft0plus7Auto;
import org.firstinspires.ftc.teamcode.subsystems.NewDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.NewIntake;
import org.firstinspires.ftc.teamcode.subsystems.NewOuttake;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

import java.util.List;

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

    private List<LynxModule> allHubs;

    private Encoder verticalSlideEncoder, horizontalSlideEncoder;

    private TouchSensor breakBeam;

    private Telemetry.Item hasSampleTelem;

    private Telemetry.Item targetAngle;

    private double maxGrabAngle = Math.toRadians(-193);
    private double minGrabAngle = Math.toRadians(-167);

    private double extensionDistance = 0;

    private final ElapsedTimer autoTimer = new ElapsedTimer();

    private double loopTime = .05;

    Vector2d holdPoint = new Vector2d(0, 0);

    double targetHeading = 0;
    double prevHeading = 0;
    ElapsedTimer loopTimer = new ElapsedTimer();

    Action grabFromSubmersible = new GrabFromSubmersible();

    boolean runAction = false;


    @Override
    public void runOpMode() throws InterruptedException {
        loopTimeTelem = telemetry.addData("Loop time:", loopTimer.milliSeconds());

        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);



        horizontalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalLeft"));
        breakBeam = hardwareMap.get(TouchSensor.class, "breakBeam");

        intake = new NewIntake(masterThread.getData(), horizontalSlideEncoder, breakBeam, blueAlliance, true, false, () -> drivetrain.getVoltage());

        drivetrain = new NewDrivetrain(masterThread.getData(), intake);
        drivetrain.setDriveState(NewDrivetrain.DriveState.FOLLOW_PATH);

        verticalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalLeft"));

        outtake = new NewOuttake(masterThread.getData(), intake, verticalSlideEncoder, blueAlliance, true, true, true, false, () -> drivetrain.getVoltage());

        vision = new VisionSubsystem(drivetrain, masterThread.getData(), blueAlliance);

        hasSampleTelem = telemetry.addData("Has sample", "");

        targetAngle = telemetry.addData("Target angle", "");

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

        drivetrain.drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));

        while ( !isStopRequested()) {
            masterThread.unThreadedUpdate();


//            double targetHeading = prevHeading+Rotation.inRange((vision.getTargetRobotPose().getHeading()-prevHeading)*.125, Math.PI, -Math.PI);
//                drivetrain.holdPoint(holdPoint.toPose(targetHeading));//vision.getTargetRobotPose()
            if (gamepad1.a) {
                intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION);
                vision.recenter();
                drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(180)));
            } else if (gamepad1.y) {
                drivetrain.cancelHoldPoint();
            }


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
        RRLeft0plus7Auto.GrabFromSubmersibleState grabFromSubmersibleState = RRLeft0plus7Auto.GrabFromSubmersibleState.SEARCHING;

        boolean searching = false;
        Action searchTurn;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (intake.getPrevIntakingState() == NewIntake.IntakingState.FINISH_EJECTING || intake.getPrevIntakingState() == NewIntake.IntakingState.START_EJECTING) {
                grabFromSubmersibleState = RRLeft0plus7Auto.GrabFromSubmersibleState.EJECTING;
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
                        grabFromSubmersibleState = RRLeft0plus7Auto.GrabFromSubmersibleState.APPROACHING_HEADING;
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
                        grabFromSubmersibleState = RRLeft0plus7Auto.GrabFromSubmersibleState.APPROACHING;

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
                        grabFromSubmersibleState = RRLeft0plus7Auto.GrabFromSubmersibleState.INTAKING;
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

                        grabFromSubmersibleState = RRLeft0plus7Auto.GrabFromSubmersibleState.RESETTING;
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

                        grabFromSubmersibleState = RRLeft0plus7Auto.GrabFromSubmersibleState.RESETTING;
                    }
                    break;
                case RESETTING:
                    if (drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(1, 1, Math.toRadians(3.5)))) {
                        grabFromSubmersibleState = RRLeft0plus7Auto.GrabFromSubmersibleState.SEARCHING;
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
