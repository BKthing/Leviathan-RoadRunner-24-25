package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.MathUtil.robotToIntakePos;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.reefsharklibrary.data.Pose2d;

import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

public class VisionSubsystem extends SubSystem {

    private final NewIntake intake;
    private final NewDrivetrain drivetrain;

    private double slidePos = 0;
    private Pose2d robotPos = new Pose2d(0, 0, 0);

    private Pose2d intakePos = new Pose2d(0, 0, 0);

    public VisionSubsystem(NewIntake intake, NewDrivetrain drivetrain, SubSystemData data) {
        super(data);

        this.intake = intake;
        this.drivetrain = drivetrain;

    }

    @Override
    public void priorityData() {
        slidePos = intake.getActualSlidePos();
        robotPos = drivetrain.getPoseEstimate();
    }

    @Override
    public void loop() {
        intakePos = robotToIntakePos(robotPos, slidePos+7.5);
    }

    @Override
    public TelemetryPacket dashboard(TelemetryPacket packet) {
        packet.fieldOverlay().setStrokeWidth(1);
        packet.fieldOverlay().setStroke("#4CAF50");
        DashboardUtil.drawRobotWithIntake(packet.fieldOverlay(), robotPos, intakePos);

        return packet;
    }
}
