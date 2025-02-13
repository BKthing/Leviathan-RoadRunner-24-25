package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.TimePose2d;
import com.reefsharklibrary.data.Vector2d;

import java.util.List;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
public class DashboardUtil {
    private static final double DEFAULT_RESOLUTION = 2.0; // distance units; presumed inches
    private static final double ROBOT_RADIUS = 9; // in

    private static final int POSE_HISTORY_LENGTH = 250;//how many prev robot poses will be displayed


    public static void drawPoseHistory(Canvas canvas, List<TimePose2d> poseHistory) {
        int length = Math.min(poseHistory.size(), POSE_HISTORY_LENGTH);
        double[] xPoints = new double[length];
        double[] yPoints = new double[length];
        for (int i = 0; i < length; i++) {
            Pose2d pose = poseHistory.get(poseHistory.size()-i-1);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawFullPoseHistory(Canvas canvas, List<TimePose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, List<Pose2d> poseList) {
        int samples = poseList.size();
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];

        for (int i = 0; i<samples; i++) {
            xPoints[i] = poseList.get(i).getX();
            yPoints[i] = poseList.get(i).getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        Vector2d frontLeft = new Vector2d(8.38582, -6.35925).rotate(pose.getHeading());
        Vector2d backLeft = new Vector2d(-8.22835, -6.35925).rotate(pose.getHeading());
        Vector2d frontRight = new Vector2d(8.38582, 6.35925).rotate(pose.getHeading());
        Vector2d backRight = new Vector2d(-8.22835, 6.35925).rotate(pose.getHeading());

        double[] xPoints = new double[5];
        double[] yPoints = new double[5];

        xPoints[0] = pose.getX()+frontLeft.getX();
        yPoints[0] = pose.getY()+frontLeft.getY();

        xPoints[1] = pose.getX()+backLeft.getX();
        yPoints[1] = pose.getY()+backLeft.getY();

        xPoints[2] = pose.getX()+backRight.getX();
        yPoints[2] = pose.getY()+backRight.getY();

        xPoints[3] = pose.getX()+frontRight.getX();
        yPoints[3] = pose.getY()+frontRight.getY();

        xPoints[4] = pose.getX()+frontLeft.getX();
        yPoints[4] = pose.getY()+frontLeft.getY();

        canvas.strokePolyline(xPoints, yPoints);


//        canvas.strokeLine(pose.getX()+frontLeft.getX(), pose.getY()+frontLeft.getY(), pose.getX()+backLeft.getX(), pose.getY()+backLeft.getY());
//        canvas.strokeLine(pose.getX()+frontRight.getX(), pose.getY()+frontRight.getY(), pose.getX()+backRight.getX(), pose.getY()+backRight.getY());
//
//        canvas.strokeLine(pose.getX()+frontLeft.getX(), pose.getY()+frontLeft.getY(), pose.getX()+frontRight.getX(), pose.getY()+frontRight.getY());
//        canvas.strokeLine(pose.getX()+backLeft.getX(), pose.getY()+backLeft.getY(), pose.getX()+backRight.getX(), pose.getY()+backRight.getY());

    }

    public static void drawIntake(Canvas canvas, Pose2d pose, double intakeDistance) {
        Vector2d frontLeft = new Vector2d(8.38582, -4.7).rotate(pose.getHeading());
        Vector2d intakeLeft = new Vector2d(intakeDistance, -4.7).rotate(pose.getHeading());
        Vector2d frontRight = new Vector2d(8.38582, 4.7).rotate(pose.getHeading());
        Vector2d intakeRight = new Vector2d(intakeDistance, 4.7).rotate(pose.getHeading());

        double[] xPoints = new double[4];
        double[] yPoints = new double[4];

        xPoints[0] = pose.getX()+frontLeft.getX();
        yPoints[0] = pose.getY()+frontLeft.getY();

        xPoints[1] = pose.getX()+intakeLeft.getX();
        yPoints[1] = pose.getY()+intakeLeft.getY();

        xPoints[2] = pose.getX()+intakeRight.getX();
        yPoints[2] = pose.getY()+intakeRight.getY();

        xPoints[3] = pose.getX()+frontRight.getX();
        yPoints[3] = pose.getY()+frontRight.getY();

        canvas.strokePolyline(xPoints, yPoints);

//        canvas.strokeLine(pose.getX()+frontLeft.getX(), pose.getY()+frontLeft.getY(), pose.getX()+intakeLeft.getX(), pose.getY()+intakeLeft.getY());
//        canvas.strokeLine(pose.getX()+frontRight.getX(), pose.getY()+frontRight.getY(), pose.getX()+intakeRight.getX(), pose.getY()+intakeRight.getY());
//        canvas.strokeLine(pose.getX()+intakeRight.getX(), pose.getY()+intakeRight.getY(), pose.getX()+intakeLeft.getX(), pose.getY()+intakeLeft.getY());

    }

    public static void drawArrow(Canvas canvas, Vector2d start, Vector2d end) {
        if (start.getX() == end.getX() && start.getY() == end.getY()) {
            return;
        }
        //rounding bc for some reason it doesn't work if i don't
        Vector2d roundedEnd = new Vector2d( (double) Math.round(end.getX()*1000)/1000, (double) Math.round(end.getY()*1000)/1000);
        canvas.strokeLine(start.getX(), start.getY(), roundedEnd.getX(), roundedEnd.getY());

        //code to draw arrow on end of the line
        double direction = start.minus(roundedEnd).getDirection();
        Vector2d arrowLeft = new Vector2d(2.5, direction+Math.toRadians(-40), true);
        Vector2d arrowRight = new Vector2d(2.5, direction+Math.toRadians(40), true);

        canvas.strokePolyline(
                new double[] {roundedEnd.getX()+arrowLeft.getX(), roundedEnd.getX(), roundedEnd.getX()+arrowRight.getX()},
                new double[] {roundedEnd.getY()+arrowLeft.getY(), roundedEnd.getY(), roundedEnd.getY()+arrowRight.getY()});
    }

    public static void drawMarker(Canvas canvas, Vector2d center, boolean active) {
        canvas.strokeLine(center.getX()+1, center.getY()+1, center.getX()-1, center.getY()-1);
        canvas.strokeLine(center.getX()-1, center.getY()+1, center.getX()+1, center.getY()-1);

        if (active) {
            canvas.strokeCircle(center.getX(), center.getY(), 2.5);
        }

    }
}
