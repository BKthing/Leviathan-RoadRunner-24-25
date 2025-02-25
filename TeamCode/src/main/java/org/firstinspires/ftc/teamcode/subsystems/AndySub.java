package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.reefsharklibrary.data.MotorPowers;
import com.reefsharklibrary.data.Vector2d;

import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

import java.util.List;

public class AndySub extends SubSystem{

    DcMotorEx FrontLeft;
    DcMotorEx FrontRight;
    DcMotorEx BackLeft;
    DcMotorEx BackRight;

    MotorPowers mp = new MotorPowers();

    public AndySub(SubSystemData data) {
        super(data);
        FrontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        FrontRight = hardwareMap.get(DcMotorEx.class, "fr");
        BackLeft = hardwareMap.get(DcMotorEx.class, "bl");
        BackRight = hardwareMap.get(DcMotorEx.class, "br");

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void priorityData() {

    }

    @Override
    public void loop() {

        mp.reset();
        mp.addVector(new Vector2d(-gamepad1.left_stick_y, gamepad1.left_stick_x));
        mp.addHeading(gamepad1.right_stick_x);

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
