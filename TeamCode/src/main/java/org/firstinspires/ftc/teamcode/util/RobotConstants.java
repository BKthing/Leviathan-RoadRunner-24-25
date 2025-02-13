package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.reefsharklibrary.data.AngVelConstraint;
import com.reefsharklibrary.data.ConstraintSet;
import com.reefsharklibrary.data.PIDCoeficients;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.VelConstraint;

@Config
public class RobotConstants {
    public static PIDCoeficients lateralPID = new PIDCoeficients(.15, .0, .02, 0, 0, 0);//.17, .0, .022,

    public static PIDCoeficients pointPID = new PIDCoeficients(.17, 0, .01, 0, 0, 0);//.2, 0, .02

    public static double f = .1;

    public static PIDCoeficients headingPID = new PIDCoeficients(.08, .0, 0.0002);//.17, .07, 0.005 //0.0013 .1

    public static PIDCoeficients headingPointPID = new PIDCoeficients(.055, 0, 0.0003);//.002

    public static Pose2d naturalDecel = new Pose2d(250, 250, Math.toRadians(600));//400, 400

    public static double trackWidth = 16;

    public static final VelConstraint velConstraint = new VelConstraint(50, 50, 50);

    public static final AngVelConstraint angVelConstraint = new AngVelConstraint(Math.toRadians(300), Math.toRadians(300), Math.toRadians(300));

    public static final ConstraintSet constraints = new ConstraintSet(lateralPID, pointPID, headingPID, headingPointPID, velConstraint, angVelConstraint, naturalDecel, trackWidth, 1.1, .15);

    public static final int cameraRows = 320;

    public static final int cameraColumns = 240;

}
