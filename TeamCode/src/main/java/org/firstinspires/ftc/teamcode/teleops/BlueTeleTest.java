package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.roadrunner.Pose2d;
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
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

import java.util.List;

@TeleOp
public class BlueTeleTest extends LinearOpMode {
    NewDrivetrain drivetrain;
    NewIntake intake;
    NewOuttake outtake;
    MasterThread masterThread;
    Telemetry.Item loopTime;

    Boolean blueAlliance = true;

    private List<LynxModule> allHubs;

    private Encoder verticalSlideEncoder, horizontalSlideEncoder;

    private DcMotorEx perpendicularWheel, parallelWheel;

    private TouchSensor breakBeam;


    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTimer loopTimer = new ElapsedTimer();
        loopTime = telemetry.addData("Loop time:", loopTimer.milliSeconds());

        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);


//        Twist2dDual<Time> testPose = new Twist2dDual<Time>(new Vector2dDual<>(new DualNum<Time>(new double[2.0, 4.1])))

        perpendicularWheel = hardwareMap.get(DcMotorEx.class, "verticalRight");
        parallelWheel = hardwareMap.get(DcMotorEx.class, "bl");


        drivetrain = new NewDrivetrain(masterThread.getData(), parallelWheel, perpendicularWheel);
        drivetrain.setDriveState(NewDrivetrain.DriveState.DRIVER_CONTROL);

        horizontalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalLeft"));
        breakBeam = hardwareMap.get(TouchSensor.class, "breakBeam");

        intake = new NewIntake(masterThread.getData(), horizontalSlideEncoder, breakBeam, blueAlliance, true, false, () -> drivetrain.getVoltage());


        verticalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalLeft"));

        outtake = new NewOuttake(masterThread.getData(), intake, verticalSlideEncoder, blueAlliance, true, true, true, false, () -> drivetrain.getVoltage());

        //its important that outtake is added after intake for update order purposes
        masterThread.addSubSystems(
                drivetrain,
                intake,
                outtake
        );


        waitForStart();
        masterThread.clearBulkCache();

        drivetrain.drive.setPoseEstimate(new Pose2d(0, 0, 0));

        while ( !isStopRequested()) {
            masterThread.unThreadedUpdate();

            parallelWheel.getCurrentPosition();

            loopTime.setValue(loopTimer.milliSeconds());

            loopTimer.reset();
        }
    }

}
