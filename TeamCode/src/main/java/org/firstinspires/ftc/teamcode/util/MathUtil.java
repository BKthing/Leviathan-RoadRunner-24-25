package org.firstinspires.ftc.teamcode.util;

import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;

public class MathUtil {

    public static double clip(double number, double min, double max) {
        if (number < min) return min;
        if (number > max) return max;
        return number;
    }

    public static com.acmerobotics.roadrunner.Pose2d toRoadRunnerPose(com.reefsharklibrary.data.Pose2d pose) {
        return new com.acmerobotics.roadrunner.Pose2d(pose.getX(), pose.getY(), pose.getHeading());
    }

    public static Pose2d toReefSharkPose(com.acmerobotics.roadrunner.Pose2d pose) {
        return new Pose2d(pose.position.x, pose.position.y, pose.heading.toDouble());
    }

    public static Pose2d robotToIntakePos(Pose2d robotPos, double slideLength) {
        return robotPos.plus(new Vector2d(slideLength, robotPos.getHeading(), true).toPose(0));
    }

}
