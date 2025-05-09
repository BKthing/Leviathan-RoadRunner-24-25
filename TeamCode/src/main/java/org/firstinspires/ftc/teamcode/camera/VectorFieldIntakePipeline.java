package org.firstinspires.ftc.teamcode.camera;

import static org.opencv.core.CvType.CV_8UC1;
import static org.opencv.imgproc.Imgproc.COLOR_GRAY2BGR;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.cameraColumns;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.cameraRows;
import static org.opencv.imgproc.Imgproc.LINE_4;

import com.reefsharklibrary.data.Vector2d;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.concurrent.atomic.AtomicBoolean;

public class VectorFieldIntakePipeline extends OpenCvPipeline {

    public enum DisplayType {
        COLOR_FILTER,
        BASE_VECTOR_FIELD,
        BLOCK_VECTOR_FIELD,
        COMBINED_VECTOR_FIELD,
        RAW_CAMERA
    }

    DisplayType displayType = DisplayType.COLOR_FILTER;

    private boolean blueAlliance = false;

    private boolean grabAllianceColor = false;

    Scalar lowB = new Scalar(95, 85, 8);
    Scalar highB = new Scalar(150, 255, 250);

    Scalar lowR = new Scalar(0, 125, 65);
    Scalar highR = new Scalar(12, 255, 255);

    Scalar lowR2 = new Scalar(198, 110, 42);
    Scalar highR2 = new Scalar(255, 255, 255);

    Scalar lowY = new Scalar(11, 64, 80);
    Scalar highY = new Scalar(65, 255, 255);

    Scalar push = new Scalar(0);
    Scalar neutral = new Scalar(127);
    Scalar pull = new Scalar(255);


    Mat largeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(64, 64));
    Mat mediumElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(32, 32));

    Mat smallElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(8, 8));


    Mat blurred = new Mat();
    Mat maskBlue = new Mat();
    Mat maskRed = new Mat();
    Mat maskRed2 = new Mat();
    Mat maskYellow = new Mat();

    Mat hsvMat = new Mat();

    Mat smallInput;
    Mat output = new Mat();

    Mat combinedVectorField = new Mat(cameraRows, cameraColumns, CV_8UC1, neutral);
    Mat baseVectorField = new Mat(cameraRows, cameraColumns, CV_8UC1);

    Mat blockVectorField, blockPushVectorField, blockPullVectorField, rawBlockPullVectorField;

    Vector2d intakePoint = new Vector2d(cameraColumns/2, cameraRows/2);

    private Vector2d targetBlockPixels = new Vector2d(0, 0);
    private Vector2d closestTargetBLockPixels = new Vector2d(0, 0);


    public AtomicBoolean hasSample = new AtomicBoolean();

    public AtomicBoolean reCenter = new AtomicBoolean();

    private boolean hasSampleValue = false;

    private final Scalar
            blue = new Scalar(0, 0, 255),
            red = new Scalar(255, 0, 0),
            yellow = new Scalar(255, 255, 0);

    public VectorFieldIntakePipeline() {
//        int fadeRange = 50/2;
//
//        for (int i = 0; i < cameraRows; i++) {
//
//        }

        hasSample.set(false);

    }


    //bgr
    @Override
    public Mat processFrame(Mat input) {
        try {
            return process(input);
        } catch (Exception e) {
            e.printStackTrace();

            throw new RuntimeException(e);
        }
    }

    private Mat process(Mat input) {

        Imgproc.blur(input, blurred, new Size(7, 7));

        Imgproc.cvtColor(blurred, hsvMat, Imgproc.COLOR_RGB2HSV);

        if (hsvMat.empty()) {
            return input;
        }

        input.copyTo(output);


        //Blue
        Core.inRange(hsvMat, lowB, highB, maskBlue);

        //Red
        Core.inRange(hsvMat, lowR, highR, maskRed);
        Core.inRange(hsvMat, lowR2, highR2, maskRed2);
        Core.add(maskRed, maskRed2, maskRed);


        //Yellow
        Core.inRange(hsvMat, lowY, highY, maskYellow);



        //Vector field
        blockVectorField = new Mat(cameraRows, cameraColumns, CV_8UC1, neutral);
        blockPushVectorField = new Mat(cameraRows, cameraColumns, CV_8UC1, neutral);
        blockPullVectorField = new Mat(cameraRows, cameraColumns, CV_8UC1, neutral);
        rawBlockPullVectorField = new Mat(cameraRows, cameraColumns, CV_8UC1, push);

        if (blueAlliance) {
            blockPullVectorField.setTo(pull, maskBlue);
            rawBlockPullVectorField.setTo(pull, maskBlue);

            if (grabAllianceColor) {
                blockPushVectorField.setTo(push, maskYellow);
            } else {
                blockPullVectorField.setTo(pull, maskYellow);
                rawBlockPullVectorField.setTo(pull, maskYellow);
            }
            blockPushVectorField.setTo(push, maskRed);
        } else {
            blockPullVectorField.setTo(pull, maskRed);
            rawBlockPullVectorField.setTo(pull, maskRed);
            if (grabAllianceColor) {
                blockPushVectorField.setTo(push, maskYellow);
            } else {
                blockPullVectorField.setTo(pull, maskYellow);
                rawBlockPullVectorField.setTo(pull, maskYellow);
            }
            blockPushVectorField.setTo(push, maskBlue);
        }



        //push cleaning
        Imgproc.dilate(blockPushVectorField, blockPushVectorField, smallElement);
        //
        Imgproc.erode(blockPushVectorField, blockPushVectorField, smallElement);
        Imgproc.erode(blockPushVectorField, blockPushVectorField, smallElement);
        Imgproc.erode(blockPushVectorField, blockPushVectorField, smallElement);
        Imgproc.erode(blockPushVectorField, blockPushVectorField, smallElement);


        //pull cleaning
        Imgproc.erode(blockPullVectorField, blockPullVectorField, smallElement);
        //
        Imgproc.dilate(blockPullVectorField, blockPullVectorField, smallElement);
        Imgproc.dilate(blockPullVectorField, blockPullVectorField, smallElement);
        Imgproc.dilate(blockPullVectorField, blockPullVectorField, smallElement);
        Imgproc.dilate(blockPullVectorField, blockPullVectorField, smallElement);


        //combining push and pull
        Core.addWeighted(blockPushVectorField, .5, blockPullVectorField, .5, 0, blockVectorField);

        Imgproc.blur(blockVectorField, blockVectorField, new Size(32, 32));
        Imgproc.blur(blockVectorField, blockVectorField, new Size(32, 32));
        Imgproc.blur(blockVectorField, blockVectorField, new Size(16, 16));
//        Imgproc.blur(blockVectorField, blockVectorField, new Size(16, 16));
        Imgproc.blur(blockVectorField, blockVectorField, new Size(8, 8));

//        for(int i = 0; i<10; i++) {
//            Imgproc.blur(blockVectorField, blockVectorField, new Size(8, 8));
//        }



        switch (displayType) {
            case COLOR_FILTER:
                output.setTo(blue, maskBlue);
                output.setTo(red, maskRed);
                output.setTo(yellow, maskYellow);
                break;
            case BLOCK_VECTOR_FIELD:
                //                output.setTo(blockVectorField);
                Imgproc.cvtColor(rawBlockPullVectorField, output, COLOR_GRAY2BGR);
                //                blockVectorField.convertTo(output, COLOR_GRAY2BGR);
                if (reCenter.compareAndSet(true, false)) {
                    intakePoint = new Vector2d(cameraColumns* .75, cameraRows*.5);
                }

                intakePoint = searchField(blockVectorField, intakePoint, 16);

                if (safeGetMat(blockPullVectorField, intakePoint.getX(), intakePoint.getY()) < 140) {
                    intakePoint = searchField(blockVectorField, new Vector2d(cameraColumns*.5, cameraRows*.5), 64);

                    if (safeGetMat(blockPullVectorField, intakePoint.getX(), intakePoint.getY()) < 140) {
                        intakePoint = searchField(blockVectorField, new Vector2d(cameraColumns*.5, cameraRows*.5), 64);

                        if (safeGetMat(blockPullVectorField, intakePoint.getX(), intakePoint.getY()) < 140) {
                            intakePoint = searchField(blockVectorField, new Vector2d(cameraColumns*.75, cameraRows*.75), 64);
                        }
                    }
                }

                int halfWidth = 20;
                int halfHeight = 20;

                //                Imgproc.rectangle(output, new Rect((int) intakePoint.getX(), (int) intakePoint.getY(), 1, 1), blue, 3);
                Imgproc.rectangle(output, new Rect ((int) MathUtil.clip((int) intakePoint.getX() - halfWidth, 0, cameraColumns), (int) MathUtil.clip((int) intakePoint.getY() - halfHeight, 0, cameraRows), halfWidth * 2, halfHeight * 2), blue, 3);

                setTargetBlockPixels(intakePoint);


                Vector2d closestIntakePoint = findClosestEdge(rawBlockPullVectorField, intakePoint, 16, 175);
                setClosestTargetBlockPixels(closestIntakePoint);


                Imgproc.line(output, new Point(closestIntakePoint.getX(), (int) MathUtil.clip((int) closestIntakePoint.getY() - halfHeight, 0, cameraRows)), new Point(closestIntakePoint.getX(), (int) MathUtil.clip((int) closestIntakePoint.getY() + halfHeight, 0, cameraRows)), blue, 3);

                boolean newHasSample = safeGetMat(blockPullVectorField, intakePoint.getX(), intakePoint.getY())>175;

                if (newHasSample != hasSampleValue) {
                    hasSampleValue = newHasSample;
                    hasSample.set(hasSampleValue);
                }

                break;
            case RAW_CAMERA:
                input.copyTo(output);
                break;
        }


        return output;
    }

    public void setDisplayType(DisplayType displayType) {
        this.displayType = displayType;
    }

    public void setBlueAlliance(boolean blueAlliance) {
        this.blueAlliance = blueAlliance;
    }

    public void setGrabAllianceColor(boolean grabAllianceColor) {
        this.grabAllianceColor = grabAllianceColor;
    }

    private Vector2d searchField (Mat mat, Vector2d startPoint, int reach) {
        double cur = safeGetMat(mat, startPoint.getX(), startPoint.getY());
        double up = safeGetMat(mat, startPoint.getX(), startPoint.getY()+reach);
        double down = safeGetMat(mat, startPoint.getX(), startPoint.getY()-reach);
        double left = safeGetMat(mat, startPoint.getX()-reach, startPoint.getY());
        double right = safeGetMat(mat, startPoint.getX()+reach, startPoint.getY());

        double max = Math.max(cur, Math.max(up, Math.max(down, Math.max(left, right))));

        if (max == cur) {
            if (reach>4) {
                return searchField(mat, startPoint, reach/2);
            } else {
                return startPoint;
            }
        } else if (max == up) {
            return searchField(mat, new Vector2d(startPoint.getX(), MathUtil.clip(startPoint.getY()+reach, 0, mat.rows()-1)), reach);
        } else if (max == down) {
            return searchField(mat, new Vector2d(startPoint.getX(), MathUtil.clip(startPoint.getY()-reach, 0, mat.rows()-1)), reach);
        } else if (max == left) {
            return searchField(mat, new Vector2d( MathUtil.clip(startPoint.getX()-reach, 0, mat.cols()-1), startPoint.getY()), reach);
        } else {
            return searchField(mat, new Vector2d(MathUtil.clip(startPoint.getX()+reach, 0, mat.cols()-1), startPoint.getY()), reach);
        }
    }

    private Vector2d findClosestEdge (Mat mat, Vector2d startPoint, int reach, int cutoff) {
        int diagReach = (int) ((double)reach*Math.sqrt(2)/2);

        double cur = safeGetMat(mat, startPoint.getX(), startPoint.getY());
        double down = safeGetMat(mat, startPoint.getX()+reach, startPoint.getY());
        double downRight = safeGetMat(mat, startPoint.getX()+diagReach, startPoint.getY()-diagReach);
        double downLeft = safeGetMat(mat, startPoint.getX()+diagReach, startPoint.getY()+diagReach);
        double right = safeGetMat(mat, startPoint.getX(), startPoint.getY()-reach);
        double left = safeGetMat(mat, startPoint.getX(), startPoint.getY()+reach);


        if (startPoint.getX() != mat.cols()-1) {
            if (down > cutoff) {
                return findClosestEdge(mat, new Vector2d(MathUtil.clip(startPoint.getX()+reach, 0, mat.cols()-1), startPoint.getY()), reach, cutoff);
            } else if (downLeft > cutoff && downLeft>downRight) {
                return findClosestEdge(mat, new Vector2d(MathUtil.clip(startPoint.getX()+diagReach, 0, mat.cols()-1), MathUtil.clip(startPoint.getY()-diagReach, 0, mat.rows()-1)), reach, cutoff);
            } else if (downRight > cutoff) {
                return findClosestEdge(mat, new Vector2d(MathUtil.clip(startPoint.getX()+diagReach, 0, mat.cols()-1), MathUtil.clip(startPoint.getY()+diagReach, 0, mat.rows()-1)), reach, cutoff);
            }
        }

        if (right > cutoff && right > cur && right >  left && startPoint.getY() != 0) {
            return findClosestEdge(mat, new Vector2d(startPoint.getX(), MathUtil.clip(startPoint.getY()-reach, 0, mat.rows()-1)), reach, cutoff);
        } else if (left > cutoff && left > cur && startPoint.getY() != mat.rows()-1) {
            return findClosestEdge(mat, new Vector2d(startPoint.getX(), MathUtil.clip(startPoint.getY()+reach, 0, mat.rows()-1)), reach, cutoff);
        }

        if (reach>4) {
                return findClosestEdge(mat, startPoint, reach/2, cutoff);
            } else {
                return startPoint;
            }

    }



    private double safeGetMat(Mat mat, double y, double x) {
        try {
            return mat.get((int) MathUtil.clip(x, 0, mat.rows()-1), (int) MathUtil.clip(y, 0, mat.cols()-1))[0];
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    public synchronized void setTargetBlockPixels(Vector2d vector2d) {
        targetBlockPixels = vector2d;
    }

    public synchronized Vector2d getTargetBlockPixels() {
        return targetBlockPixels;
    }

    public synchronized void setClosestTargetBlockPixels(Vector2d vector2d) {
        closestTargetBLockPixels = vector2d;
    }

    public synchronized Vector2d getClosestTargetBlockPixels() {
        return closestTargetBLockPixels;
    }
}
