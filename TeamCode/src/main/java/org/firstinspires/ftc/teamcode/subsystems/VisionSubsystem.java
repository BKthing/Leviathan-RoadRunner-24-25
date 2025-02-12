package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.MathUtil.robotToIntakePos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.cameraColumns;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.cameraRows;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.reefsharklibrary.data.Pose2d;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.camera.VectorFieldIntakePipeline;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

public class VisionSubsystem extends SubSystem {
    private final NewDrivetrain drivetrain;

    private final OpenCvCamera webcam;

    private final VectorFieldIntakePipeline pipeline;

    private Pose2d intakePose = new Pose2d(0, 0, 0);

    private Boolean blueAlliance;



    public VisionSubsystem(NewDrivetrain drivetrain, SubSystemData data, Boolean blueAlliance) {
        super(data);

        this.drivetrain = drivetrain;

        pipeline = new VectorFieldIntakePipeline();

        this.blueAlliance = blueAlliance != null ? blueAlliance : true;

        pipeline.setBlueAlliance(this.blueAlliance);
        pipeline.setDisplayType(VectorFieldIntakePipeline.DisplayType.BLOCK_VECTOR_FIELD);

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"));
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam.setPipeline(pipeline);

                webcam.startStreaming(cameraRows, cameraColumns);//, OpenCvCameraRotation.UPSIDE_DOWN);
                FtcDashboard.getInstance().startCameraStream(webcam, 10);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error: ", errorCode);
            }
        });

    }

    @Override
    public void priorityData() {
        intakePose = drivetrain.getIntakePoseEstimate();
    }

    @Override
    public void loop() {

    }

    @Override
    public TelemetryPacket dashboard(TelemetryPacket packet) {
//        packet.fieldOverlay().setStrokeWidth(1);
//
//        packet.fieldOverlay().setStroke("#0a0a0f");//black
//        DashboardUtil.drawIntake(packet.fieldOverlay(), robotPos, slidePos);
//
//
//        packet.fieldOverlay().setStroke("#3F51B5");//blue
//        DashboardUtil.drawRobot(packet.fieldOverlay(), robotPos);

        return packet;
    }
}
