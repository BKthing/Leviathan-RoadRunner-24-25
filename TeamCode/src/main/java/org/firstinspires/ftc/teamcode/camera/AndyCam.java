package org.firstinspires.ftc.teamcode.camera;

import android.media.Image;

import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class AndyCam extends OpenCvPipeline {

    Mat output = new Mat();
    Mat savedImg = new Mat();

    Mat inRange = new Mat();

    Mat hsvMat = new Mat();

    Scalar min = new Scalar(95, 40, 5);
    Scalar max = new Scalar(160, 255, 255);

    Scalar blue = new Scalar(67, 67, 67);



    @Override
    public Mat processFrame(Mat input) {
        Imgproc.blur(input, savedImg, new Size(6, 6));
        Imgproc.cvtColor(savedImg, hsvMat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsvMat, min, max, inRange);

        input.copyTo(output);
        output.setTo(blue, inRange);

        return output;
    }
}
