package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.util.MathUtil.toRoadRunnerPose;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.NewDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.NewIntake;
import org.firstinspires.ftc.teamcode.subsystems.NewOuttake;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

import java.util.List;

@TeleOp
public class CameraPickupTest extends LinearOpMode {
    NewDrivetrain drivetrain;
    NewIntake intake;
    NewOuttake outtake;
    VisionSubsystem vision;
    MasterThread masterThread;
    Telemetry.Item loopTime;

    Boolean blueAlliance = true;

    private List<LynxModule> allHubs;

    private Encoder verticalSlideEncoder, horizontalSlideEncoder;

    private TouchSensor breakBeam;

    private Telemetry.Item hasSampleTelem;

    private Telemetry.Item targetAngle;


    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTimer loopTimer = new ElapsedTimer();
        loopTime = telemetry.addData("Loop time:", loopTimer.milliSeconds());

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


        waitForStart();
        masterThread.clearBulkCache();

        drivetrain.drive.setPoseEstimate(new Pose2d(0, 0, 0));

        while ( !isStopRequested()) {
            masterThread.unThreadedUpdate();


//            if (gamepad1.left_bumper) {
                drivetrain.holdPoint(new com.reefsharklibrary.data.Pose2d(0, 0, 0));//vision.getTargetRobotPose()
//            } else {
//                drivetrain.cancelHoldPoint();
//            }

            loopTime.setValue(loopTimer.milliSeconds());

            loopTimer.reset();
        }
    }

}
