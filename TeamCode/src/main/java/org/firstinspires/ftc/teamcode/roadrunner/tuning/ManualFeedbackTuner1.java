//package org.firstinspires.ftc.teamcode.roadrunner.tuning;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.canvas.Canvas;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive2;
//import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;
//import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;
//import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
//
//import java.util.Arrays;
//import java.util.List;
//@Disabled
//
//public final class ManualFeedbackTuner1 extends LinearOpMode {
//    public static double DISTANCE = 64;
//
//    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
//    private DcMotorEx perpendicularWheel, parallelWheel;
//
//
//    private List<DcMotorEx> drivetrainMotors;
//
//    private List<Double> lastPowers = Arrays.asList(0.0, 0.0, 0.0, 0.0);
//    @Override
//    public void runOpMode() throws InterruptedException {
//        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive2.class)) {
//            perpendicularWheel = hardwareMap.get(DcMotorEx.class, "verticalRight");
//            parallelWheel = hardwareMap.get(DcMotorEx.class, "bl");
//
//            MecanumDrive2 drive = new MecanumDrive2(hardwareMap, new Pose2d(0, 0, 0));
//            frontLeft = hardwareMap.get(DcMotorEx.class, "fl");//ex 0
//            frontRight = hardwareMap.get(DcMotorEx.class, "fr");//
//            backLeft = hardwareMap.get(DcMotorEx.class, "bl");
//            backRight = hardwareMap.get(DcMotorEx.class, "br");//ex 1
//
//            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//
//            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//            drivetrainMotors = Arrays.asList(frontLeft, backLeft, backRight, frontRight);
//
//            telemetry.setAutoClear(false);
//
////            if (drive.localizer instanceof TwoDeadWheelLocalizer) {
////                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
////                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
////                }
////            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
////                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
////                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
////                }
////            }
//            waitForStart();
//            drive.pinpoint.setPosition(new Pose2d(0, 0, 0));
//
//            while (opModeIsActive()) {
//                Action path =
//                    drive.actionBuilder(new Pose2d(0, 0, 0))
//                            .lineToX(DISTANCE)
//                            .lineToX(0)
//                            .build();
//
//                FtcDashboard dashboard = FtcDashboard.getInstance();
//                Canvas canvas = new Canvas();
//                path.preview(canvas);
//
//                boolean active = true;
//
//                while (active && !Thread.currentThread().isInterrupted()) {
//                    TelemetryPacket packet = new TelemetryPacket();
//                    packet.fieldOverlay().getOperations().addAll(canvas.getOperations());
//                    drive.updatePoseEstimate();
//                    active = path.run(packet);
//
//                    setDrivePower(drive.getDrivePowers());
//
//                    dashboard.sendTelemetryPacket(packet);
//
//
//                    telemetry.update();
//
//
//
//
//
//                }
//
//            }
//        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
//            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));
//
//            if (drive.localizer instanceof TwoDeadWheelLocalizer) {
//                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
//                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
//                }
//            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
//                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
//                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
//                }
//            }
//            waitForStart();
//
//            while (opModeIsActive()) {
//                Actions.runBlocking(
//                    drive.actionBuilder(new Pose2d(0, 0, 0))
//                            .lineToX(DISTANCE)
//                            .lineToX(0)
//                            .build());
//            }
//        } else {
//            throw new RuntimeException();
//        }
//    }
//    public void setDrivePower(List<Double> powers) {
////        List<Double> powers = motorPowers.getNormalizedVoltages(voltage);
//
//        for (int i = 0; i<4; i++) {
//            if ((lastPowers.get(i) == 0 && powers.get(i) != 0) || (lastPowers.get(i) != 0 && powers.get(i) == 0) || (Math.abs(powers.get(i)-lastPowers.get(i))>.1)) {
//                int finalI = i;
////                hardwareQueue.add(() -> drivetrainMotors.get(finalI).setPower(powers.get(finalI)));
//                drivetrainMotors.get(finalI).setPower(powers.get(finalI));
//            }
//        }
//
//        lastPowers = powers;
//    }
//}
