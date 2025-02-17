package org.firstinspires.ftc.teamcode.camera;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.cameraColumns;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.cameraRows;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@TeleOp
public class OpenCVTestTele extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        OpenCvCamera webcam;

        VectorFieldIntakePipeline pipeline = new VectorFieldIntakePipeline();

        pipeline.setBlueAlliance(true);
        pipeline.setDisplayType(VectorFieldIntakePipeline.DisplayType.RAW_CAMERA);
        // copy and pasted from last year's code, im not sure what "robot" or "hwMap" is this yr
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"));
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam.setPipeline(pipeline);

//                webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
                webcam.startStreaming(cameraRows, cameraColumns);//, OpenCvCameraRotation.UPSIDE_DOWN);
                FtcDashboard.getInstance().startCameraStream(webcam, 10);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error: ", errorCode);
            }
        });

        waitForStart();
        while (!isStopRequested()) {

        }
    }
}
