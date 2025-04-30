package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.cameraColumns;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.cameraRows;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.reefsharklibrary.data.MotorPowers;
import com.reefsharklibrary.data.Vector2d;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.camera.AndyCam;
import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.List;

@Disabled
public class AndySub extends SubSystem{

    DcMotorEx FrontLeft;
    DcMotorEx FrontRight;
    DcMotorEx BackLeft;
    DcMotorEx BackRight;

    private final OpenCvCamera webcam;

    AndyCam andyCam = new AndyCam();

    MotorPowers mp = new MotorPowers();

    public AndySub(SubSystemData data) {
        super(data);
        FrontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        FrontRight = hardwareMap.get(DcMotorEx.class, "fr");
        BackLeft = hardwareMap.get(DcMotorEx.class, "bl");
        BackRight = hardwareMap.get(DcMotorEx.class, "br");

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"));
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam.setPipeline(andyCam);

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

    }

    @Override
    public void loop() {

        mp.reset();
        mp.addHeading(-gamepad1.right_stick_x);
        mp.addVector(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x));

        setMp(mp);
    }

    private void setMp(MotorPowers motorpowers){
        List<Double> powers = motorpowers.getRawVoltages();
        FrontLeft.setPower(powers.get(0));
        BackLeft.setPower(powers.get(1));
        BackRight.setPower(powers.get(2));
        FrontRight.setPower(powers.get(3));
    }
}
