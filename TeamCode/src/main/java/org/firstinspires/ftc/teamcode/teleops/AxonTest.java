package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PWMOutputControllerEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.reefsharklibrary.misc.ElapsedTimer;


import org.firstinspires.ftc.ftccommon.internal.manualcontrol.commands.AnalogCommands;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.NewIntake;
import org.firstinspires.ftc.teamcode.util.MathUtil;

@TeleOp
public class AxonTest extends LinearOpMode {
    private Servo leftIntakeServo, rightIntakeServo, clawServo, leftOuttake, rightOuttake, clawPitchServo;
    private CRServo leftSpinnerServo, rightSpinnerServo;
    private double LcurPos, RcurPos;
    private NormalizedColorSensor colorSensor;

    private TouchSensor breakBeam;
    Telemetry.Item colorTelem;
    Telemetry.Item sampleColorTelem;

    Telemetry.Item breakBeamTelem;

    Telemetry.Item loopTime;

    Telemetry.Item posTelem;

    Telemetry.Item pitchPosTelem;

    Telemetry.Item servoBusCurrentTelem;

    ElapsedTimer timer = new ElapsedTimer();

    LynxModule myRevHub;
    private  LynxGetADCCommand.Channel servoChannel;
    private  LynxGetADCCommand servoCommand;
    LynxGetADCResponse servoResponse;
    private double servoBusCurrent;

    @Override
    public void runOpMode() throws InterruptedException {
        servoChannel = LynxGetADCCommand.Channel.SERVO_CURRENT;
        servoCommand = new LynxGetADCCommand(myRevHub, servoChannel, LynxGetADCCommand.Mode.ENGINEERING);
        myRevHub = hardwareMap.get(LynxModule.class, "Expansion Hub 3");
        servoBusCurrent = getServoBusCurrent();
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        servoBusCurrentTelem = telemetry.addData("Servo Bus Current", "");
        colorTelem = telemetry.addData("Color RGB", "");
        sampleColorTelem = telemetry.addData("Sample Color", "");
        loopTime = telemetry.addData("Loop Time", "");


        leftIntakeServo = hardwareMap.get(Servo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(Servo.class, "rightIntakeServo");



        leftIntakeServo.setDirection(Servo.Direction.REVERSE);



        leftSpinnerServo = hardwareMap.get(CRServo.class, "leftSpinnerServo");
        rightSpinnerServo = hardwareMap.get(CRServo.class, "rightSpinnerServo");

        leftSpinnerServo.setDirection(DcMotorSimple.Direction.REVERSE);


        breakBeam = hardwareMap.get(TouchSensor.class, "breakBeam");

        telemetry.setAutoClear(false);

        breakBeamTelem = telemetry.addData("Block", true);

        leftOuttake = hardwareMap.get(Servo.class, "leftOuttakeServo");
        rightOuttake = hardwareMap.get(Servo.class, "rightOuttakeServo");

        clawServo = hardwareMap.get(Servo.class, "clawServo"); // ex hub 4


        rightOuttake.setDirection(Servo.Direction.REVERSE);

        clawPitchServo = hardwareMap.get(Servo.class, "clawPitchServo"); // control hub 1

        posTelem = telemetry.addData("Pos", "");
        pitchPosTelem = telemetry.addData("Pitch Pos", "");
//        leftOuttake.scaleRange(.34, .965);
//        rightOuttake.scaleRange(1-.965, 1-.34);
//
//        rightIntakeServo.setDirection(Servo.Direction.REVERSE);

//
//        clawWristRoll.scaleRange(.34, .965);
//        clawWristPitch.scaleRange(.34, .965);


        double targetPos = 0;

        double targetPitchPos = 0.1;
//
//        double clawRollPos = .3;

        waitForStart();
        while (!isStopRequested()) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

//            if (gamepad2.a) {
//                if (colors.red > .006 && colors.green < .01) {
//                    sampleColorTelem.setValue("red");
//                    leftIntake.setPower(.8);
//                    rightIntake.setPower(.8);
//                }
//                else {
//                    leftIntake.setPower(-.8);
//                    rightIntake.setPower(-.8);
//                }
//
//            }
//            else if (gamepad2.b) {
//                leftIntake.setPower(.8);
//                rightIntake.setPower(.8);
//            }
//            else {
//                leftIntake.setPower(0);
//                rightIntake.setPower(0);
//            }

            colorTelem.setValue(colors.red + " " + colors.green + " " + colors.blue);

            if (colors.green > .013) {
//            throw new RuntimeException("Not yellow");
                sampleColorTelem.setValue("yellow");
            } else if (colors.red > .004) { //&& colors.green < .012) {
//            throw new RuntimeException("Not red");
                sampleColorTelem.setValue("red");
            } else if (colors.blue > .0055) {
                sampleColorTelem.setValue("blue");
            }

            else {
                sampleColorTelem.setValue("no sample");
            }

//            if (colorSensor.argb() == Color.rgb(0, 0, 0)){
//
//            }


//            if (gamepad2.y) {
//                leftIntakeServo.setPosition(.9);
//                rightIntakeServo.setPosition(.9);
//            }
//            else {
//                leftIntakeServo.setPosition(.07);
//                rightIntakeServo.setPosition(.07);
//            }
//
            if (gamepad2.a) {
                leftSpinnerServo.setPower(.6);
                rightSpinnerServo.setPower(.6);
            }  else {
                leftSpinnerServo.setPower(0);
                rightSpinnerServo.setPower(0);
            }

            breakBeamTelem.setValue(breakBeam.isPressed());


//            if (gamepad2.a) {
//                claw.setPosition(.35); //open
//            }
//
//            else if (gamepad2.b) {
//                claw.setPosition(.1); //close
//            }
//
//            if (gamepad2.x) {
//                hangDeploy.setPosition(.4); // deploy
//            }
//           else  if (gamepad2.y) {
//                hangDeploy.setPosition(.2); // not deploy
//            }

           if (Math.abs (gamepad2.left_stick_y) > .1 ) {
//               clawWristPitch.setPosition(targetPos);
//               leftIntakeServo.setPosition( targetPos + .006);
//               rightIntakeServo.setPosition(targetPos);

               targetPos = MathUtil.clip(targetPos + gamepad2.left_stick_y * .001 * timer.milliSeconds(), 0, 1);

               leftOuttake.setPosition( targetPos);
               rightOuttake.setPosition(targetPos);
           }

           if (gamepad2.dpad_up) {
               targetPos = 1;
               leftOuttake.setPosition(targetPos);
               rightOuttake.setPosition(targetPos);
           } else if (gamepad2.dpad_right) {
               targetPos = .5;
               leftOuttake.setPosition(targetPos);
               rightOuttake.setPosition(targetPos);
           } else if (gamepad2.dpad_down) {
               targetPos = 0;
               leftOuttake.setPosition(targetPos);
               rightOuttake.setPosition(targetPos);
           }

           if (gamepad2.left_bumper) {
               clawServo.setPosition(0);
           } else if (gamepad2.right_bumper) {
               clawServo.setPosition(.5);
           }

            if (Math.abs (gamepad2.right_stick_y) > .1 ) {
//               clawWristPitch.setPosition(targetPos);
//               leftIntakeServo.setPosition( targetPos + .006);
//               rightIntakeServo.setPosition(targetPos);

                targetPitchPos = MathUtil.clip(targetPitchPos + gamepad2.right_stick_y * .0002 * timer.milliSeconds(), 0, 1);

                clawPitchServo.setPosition(targetPitchPos);
            }


//
//           if (Math.abs (gamepad2.right_stick_x) > .1) {
//               clawWristRoll.setPosition(clawRollPos);
//               clawRollPos = MathUtil.clip(clawRollPos + gamepad2.right_stick_x * .0002 * timer.milliSeconds(), 0, 1);
//           }




//            leftIntakeServo.setPosition( targetPos + .006);
//            rightIntakeServo.setPosition(targetPos);



//
            servoBusCurrent = getServoBusCurrent();
            servoBusCurrentTelem.setValue(servoBusCurrent);
            posTelem.setValue(targetPos);
            pitchPosTelem.setValue(targetPitchPos);

            loopTime.setValue(timer.milliSeconds());

            telemetry.update();

            timer.reset();
        }


    }
    double getServoBusCurrent()
    {

        try
        {
            servoResponse = servoCommand.sendReceive();
            return servoResponse.getValue() / 1000.0;    // return value in Amps
        }
        catch (InterruptedException | RuntimeException | LynxNackException e)
        {
        }
        return 999;
    }

}

