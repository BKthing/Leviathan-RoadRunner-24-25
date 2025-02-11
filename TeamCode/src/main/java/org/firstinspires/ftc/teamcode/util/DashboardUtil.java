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
        canvas.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS);
        Vector2d v = new Vector2d(Math.cos(pose.getHeading()), (Math.sin(pose.getHeading()))).multiply(ROBOT_RADIUS);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }

    public static void drawRobotWithIntake(Canvas canvas, Pose2d pose, Pose2d intakePos) {
        canvas.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS);
        Vector2d v = new Vector2d(Math.cos(pose.getHeading()), (Math.sin(pose.getHeading()))).multiply(ROBOT_RADIUS);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);

        canvas.strokeLine(pose.getX(), pose.getY(), intakePos.getX(), intakePos.getY());
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
