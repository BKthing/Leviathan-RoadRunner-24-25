package org.firstinspires.ftc.teamcode.roadrunner;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.zyxOrientation;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.roadrunner.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.roadrunner.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.roadrunner.messages.TwoDeadWheelInputsMessage;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class MecanumDrive2 {
    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        // drive model parameters
        public double inPerTick = 1;
        public double lateralInPerTick = inPerTick;
        public double trackWidthTicks = 9.533;

        // feedforward parameters (in tick units)
        public double kS = 1.2986808744253784;//1.3174464421787597;
        public double kV = 0.12978478742802999;//0.12880689268881115;
        public double kA = 0.03; //.01

        // path profile parameters (in inches)
        public double maxWheelVel = 60;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 60;

        // turn profile parameters (in radians)
        public double maxAngVel = 3*Math.PI; // shared with path
        public double maxAngAccel = 3*Math.PI;

        // path controller gains
        public double axialGain = 1;
        public double lateralGain = 1;
        public double headingGain = 1; // shared with turn

        public double axialVelGain = 0;
        public double lateralVelGain = 0;
        public double headingVelGain = 0; // shared with turn

        public boolean usePinpointIMUForTuning = true;

    }
    public GoBildaPinpointDriverRR pinpoint;

    public static Params PARAMS = new Params();

    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public final VoltageSensor voltageSensor;

    public final LazyImu lazyImu;

    public  Localizer localizer;

    public Pose2d pose;
    public PoseVelocity2d robotVelRobot;

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    private List<Double> motorPowers = new ArrayList<>();

    private double voltage = 13;
    public MecanumDrive2(HardwareMap hardwareMap, Pose2d pose) {
        this(hardwareMap, hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint"), pose);

        pinpoint.setEncoderResolution(GoBildaPinpointDriverRR.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setOffsets(-78.2, -120.7);

        pinpoint.resetPosAndIMU();

    }

    public MecanumDrive2(HardwareMap hardwareMap, GoBildaPinpointDriverRR pinpoint, Pose2d pose) {
        this.pose = pose;
        this.pinpoint = pinpoint;


        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx.class, "fl");
        leftBack = hardwareMap.get(DcMotorEx.class, "bl");
        rightBack = hardwareMap.get(DcMotorEx.class, "br");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html


        voltageSensor = hardwareMap.voltageSensor.iterator().next();


        FlightRecorder.write("MECANUM_PARAMS", PARAMS);

        FlightRecorder.write("PINPOINT_PARAMS",PARAMS);

        if (PARAMS.usePinpointIMUForTuning) {
            lazyImu = new LazyImu(hardwareMap, "pinpoint", new RevHubOrientationOnRobot(zyxOrientation(0, 0, 0)));
        } else {
            lazyImu = new LazyImu(hardwareMap, "controlImu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));
        }

//        localizer = new TwoDeadWheelLocalizer(hardwareMap, lazyImu.get(), parallelEncoder, perpendicularEncoder, PARAMS.inPerTick);


        updatePoseEstimate();
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        motorPowers = Arrays.asList(wheelVels.leftFront.get(0) / maxPowerMag, wheelVels.leftBack.get(0) / maxPowerMag, wheelVels.rightBack.get(0) / maxPowerMag, wheelVels.rightFront.get(0) / maxPowerMag);//frontLeft, backLeft, backRight, frontRight
    }

    public List<Double> getDrivePowers() {
        return motorPowers;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                motorPowers = Arrays.asList(0.0, 0.0, 0.0, 0.0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            motorPowers = Arrays.asList(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);


            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                motorPowers = Arrays.asList(0.0, 0.0, 0.0, 0.0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            motorPowers = Arrays.asList(feedforward.compute(wheelVels.leftFront) / voltage, feedforward.compute(wheelVels.leftBack) / voltage, feedforward.compute(wheelVels.rightBack) / voltage, feedforward.compute(wheelVels.rightFront) / voltage);

            Canvas c = p.fieldOverlay();

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public void updatePoseEstimate(com.reefsharklibrary.data.Pose2d poseEstimate, com.reefsharklibrary.data.Pose2d poseVelocity) {
        pose = MathUtil.toRoadRunnerPose(poseEstimate);

        estimatedPoseWriter.write(new PoseMessage(pose));

        robotVelRobot = new PoseVelocity2d(new Vector2d(poseVelocity.getX(), poseVelocity.getY()), poseVelocity.getHeading());
    }

    public void updatePoseEstimate(Twist2d poseDelta, PoseVelocity2d poseVelocity) {

        pose = pose.plus(poseDelta);

        estimatedPoseWriter.write(new PoseMessage(pose));

        robotVelRobot = poseVelocity;
    }

    public void updatePoseEstimate(Pose2d pose, PoseVelocity2d poseVelocity) {

        this.pose = pose;

        estimatedPoseWriter.write(new PoseMessage(pose));

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));
//        FlightRecorder.write("PINPOINT_RAW_POSE",new PinpointDrive.FTCPoseMessage(pinpoint.getPosition()));

        FlightRecorder.write("PINPOINT_RAW_POSE",new FTCPoseMessage(new Pose2D(DistanceUnit.INCH, pose.position.x, pose.position.y, AngleUnit.RADIANS, pose.heading.toDouble())));
        FlightRecorder.write("PINPOINT_STATUS",pinpoint.getDeviceStatus());

//        FlightRecorder.write("TWO_DEAD_WHEEL_INPUTS", new TwoDeadWheelInputsMessage(poseVelocity., perpPosVel, angles, angularVelocity));

//        FlightRecorder.write("PINPOINT_STATUS",pinpoint.getDeviceStatus());

        robotVelRobot = poseVelocity;

    }

    public void updatePoseEstimate() {

        pinpoint.update();
        pose = pinpoint.getPositionRR();



        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));
        FlightRecorder.write("PINPOINT_RAW_POSE",new FTCPoseMessage(new Pose2D(DistanceUnit.INCH, pose.position.x, pose.position.y, AngleUnit.RADIANS, pose.heading.toDouble())));

//        FlightRecorder.write("PINPOINT_RAW_POSE",new PinpointDrive.FTCPoseMessage(pinpoint.getPosition()));
        FlightRecorder.write("PINPOINT_STATUS",pinpoint.getDeviceStatus());

        robotVelRobot = pinpoint.getVelocityRR();

    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    public static final class FTCPoseMessage {
        public long timestamp;
        public double x;
        public double y;
        public double heading;

        public FTCPoseMessage(Pose2D pose) {
            this.timestamp = System.nanoTime();
            this.x = pose.getX(DistanceUnit.INCH);
            this.y = pose.getY(DistanceUnit.INCH);
            this.heading = pose.getHeading(AngleUnit.RADIANS);
        }
    }
}
