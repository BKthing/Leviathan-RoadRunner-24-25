package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.cameraColumns;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.cameraRows;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    private Vector2d targetSamplePose = new Vector2d(0, 0);

    private Pose2d targetRobotPose = new Pose2d(0, 0, 0);

    private Vector2d sampleRobotDiff = new Vector2d(0, 0);

    private double intakeHeight = 0;

    private Boolean blueAlliance;

    private final Telemetry.Item visionsTelem;

    private boolean hasSampleVal = false;

    private boolean recenter = false;



    public VisionSubsystem(NewDrivetrain drivetrain, SubSystemData data, Boolean blueAlliance, boolean grabAllianceColor) {
        super(data);

        this.drivetrain = drivetrain;

        pipeline = new VectorFieldIntakePipeline();

        this.blueAlliance = blueAlliance != null ? blueAlliance : true;

        this.pipeline.setGrabAllianceColor(grabAllianceColor);

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

        visionsTelem = telemetry.addData("Camera", 0);

    }

    @Override
    public void priorityData() {
        intakePose = drivetrain.getIntakePoseEstimate();
        intakeHeight = drivetrain.getIntakeY();
        hasSampleVal = pipeline.hasSample.get();
    }

    @Override
    public void loop() {
        if (recenter) {
            recenter = false;
            pipeline.reCenter.set(true);
        }

        if (hasSampleVal) {
            Vector2d relCords = pixelToRelFieldCords(pipeline.getClosestTargetBlockPixels());
            targetSamplePose = intakePose.getVector2d().plus(relCords.rotate(intakePose.getHeading()));

            visionsTelem.setValue(relCords.getX() + " angle: " + relCords.getY());

            Pose2d driveTrainPoseEstimate = drivetrain.getPoseEstimate();

            sampleRobotDiff = targetSamplePose.minus(driveTrainPoseEstimate.getVector2d());

            targetRobotPose = driveTrainPoseEstimate.getVector2d().toPose(targetSamplePose.minus(drivetrain.getPoseEstimate().getVector2d()).getDirection());

        }



    }

    @Override
    public TelemetryPacket dashboard(TelemetryPacket packet) {


        packet.fieldOverlay().setStrokeWidth(1);
//
        packet.fieldOverlay().setStroke("#FFDE21");//yellow

        if (hasSample()) {
            DashboardUtil.drawMarker(packet.fieldOverlay(), targetSamplePose, true);
        }
//        DashboardUtil.drawIntake(packet.fieldOverlay(), robotPos, slidePos);
//
//
//        packet.fieldOverlay().setStroke("#3F51B5");//blue
//        DashboardUtil.drawRobot(packet.fieldOverlay(), robotPos);

        packet.fieldOverlay().setStroke("#4CAF50");//green


        DashboardUtil.drawRobot(packet.fieldOverlay(), targetRobotPose);


        return packet;
    }

    private Vector2d pixelToRelFieldCords(Vector2d targetBlockPixels) {
        double horizontalAngle = Math.toRadians(((targetBlockPixels.getY()-cameraRows/2)/cameraRows)*60.45786);
        double verticalAngle = Math.toRadians(((cameraColumns/2-targetBlockPixels.getX())/cameraColumns)*44.36924 + 30.87+6);

        double xOffset = Math.tan(verticalAngle)*(intakeHeight)+1;//Math.tan(verticalAngle)*(intakeHeight-1)+1

        double hypotenuse = Math.sqrt(xOffset*xOffset+(intakeHeight)*(intakeHeight));

        double yOffset = Math.tan(horizontalAngle)*hypotenuse;
//        double xOffset =
        return new Vector2d(xOffset, yOffset);
    }

    public boolean hasSample() {
        return hasSampleVal;
    }

    public Vector2d getTargetSamplePose() {
        return targetSamplePose;
    }

    public Pose2d getTargetRobotPose() {
        return targetRobotPose;
    }

    public Vector2d getSampleRobotDiff() {
        return sampleRobotDiff;
    }

    public void recenter() {
        recenter = true;
    }
}
