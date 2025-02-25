package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;
import android.graphics.Color;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.reefsharklibrary.misc.ElapsedTimer;
import com.reefsharklibrary.robotControl.ReusableHardwareAction;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.teamcode.PassData;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

public class NewIntake extends SubSystem {

    private final LynxModule myRevHub;
//    private final LynxGetADCCommand.Channel servoChannel;
//    private final LynxGetADCCommand servoCommand;
    private double servoBusCurrent = 0;
    private double updatedServoBusCurrent = 0;
    private boolean changedServoBusCurrent = false;

    public enum IntakeState {
        EXTENDING,
        DROPPING_INTAKE,
        INTAKING,
        RETRACTING_INTAKE,
        RETRACTING,
        WAITING_AFTER_RETRACTING,
        WAITING_FOR_TRANSFER,
        WAITING_FOR_TRANSFERRING,
        TRANSFERRING,
        RESTING,
        MOVE_SLIDES_MORE_IN,
        RESET_SLIDES
    }

    private IntakeState intakeState = IntakeState.RESTING;

    private IntakeState newIntakeState = intakeState;

    private boolean updateIntakeState = false;


    public enum IntakingState {
        START_INTAKING,
        INTAKING,
        INTAKING_A_LITTLE_MORE,
        INTAKING_SPIN_OUT,
        INTAKING_IN_AGAIN,
        FINISH_INTAKING,
        HOLDING_SAMPLE,
        MANUAL_EJECTING,
        START_EJECTING,
        FINISH_EJECTING,

        START_REINTAKING,
        FINISH_REINTAKING,

        START_UNJAMMING,
        UNJAMMING_SPIN_OUT,
        UNJAMMING_SPIN_IN,
        UNJAMMING_FINISH_SPIN_IN,

        SERVO_STALL_START_UNJAMMING,
        SERVO_STALL_UNJAMMING_SPIN_OUT,


        START_PARTIAL_GRAB,
        INTAKING_PARTIAL_GRAB,

        START_EJECTING_PARTIAL_GRAB,
        EJECTING_PARTIAL_GRAB,

        IDLE,


    }

    private IntakingState intakingState = IntakingState.IDLE;

    private IntakingState newIntakingState = intakingState;

    private IntakingState prevIntakingState = intakingState;

    private boolean updateIntakingState = false;

    private final ElapsedTimer intakeTimer = new ElapsedTimer();

    private final ElapsedTimer intakingTimer = new ElapsedTimer();

    private final Telemetry.Item slidePosTelem;

    private double voltage = 13;


    public enum SampleColor{
        RED,
        BLUE,
        YELLOW,
        NONE
    }

    SampleColor sampleColor = SampleColor.NONE;

    public enum ToIntakeState {
        EXTEND_THEN_DROP_INTAKE,
        DROP_INTAKE,
        DROP_INTAKE_AUTO_SHOVE_HEIGHT,
        RAISE_INTAKE,
        PARTIAL_RAISE_INTAKE,
        RAISE_TO_AUTO_HEIGHT,
        SEARCH_POSITION,
        RETRACT,
        RETRACT_AND_STOP_INTAKING,
        IDLE
    }

    private ToIntakeState toIntakeState = ToIntakeState.IDLE;

    private ToIntakeState newToIntakeState = toIntakeState;

    private boolean changedToIntakeState = false;

    public Boolean blueAlliance;


    public enum HorizontalSlide {
        //18.9 max
        EXTRA_IN(-.5),
        IN(-.2),
        AUTO_PRESET1(13.5),
        AUTO_PRESET2(4),
        CLOSE(7),
        MEDIUM(12),
        FAR(16);//17

        public final double length;
        HorizontalSlide(double length) {this.length = length;}
    }

    private double targetSlidePos = 0;
    private double newTargetSlidePos;
    private boolean changedTargetSlidePos = false;

    private double slidePos = 0;//inches
    private double prevSlideError = 0;


    private int slideTicks = 0;

    private final Encoder horizontalSlideEncoder;

    private final DcMotorEx horizontalLeftMotor, horizontalRightMotor;

    private double actualMotorPower = 0;

    private final DoubleSupplier getVoltage;

    public enum IntakePos {
        UP(.75),//.69
        AUTO_HEIGHT(.55),//.1),
        AUTO_SHOVE_HEIGHT(.39),
        PARTIAL_UP(.5),//.11),
        SEARCH(.53),
        DOWN(.42);//.16);//.05

        public final double pos;
        IntakePos(double pos) {this.pos = pos;}
    }
    private double targetIntakePos = IntakePos.UP.pos;
    private double actualIntakePos = -1;

    private double targetIntakeSpeed = 0;
    private double actualIntakeSpeed = 0;


    private final Servo leftIntakeServo, rightIntakeServo;
    private final CRServo leftSpinnerServo, rightSpinnerServo;

    private NormalizedColorSensor colorSensor;

    private TouchSensor breakBeam;

    private boolean changedIsBreakBeam = false;
    private boolean updatedIsBreakBeam = false;
    private boolean isBreakBeam = false;
    private boolean prevIsBreakBeam = false;

    private final ElapsedTimer slideTimer = new ElapsedTimer();
    private final ElapsedTimer timeSinceLastStall = new ElapsedTimer();
    private int stallCount = 0;
    private final ElapsedTimer servoStallTimer = new ElapsedTimer();

    private boolean transfer = false;

    private final boolean teleOpControls;

    NormalizedRGBA colors = new NormalizedRGBA();

    NormalizedRGBA newColors;

    private boolean nextCheckColor = false;
    private boolean checkColor = false;

    float hsvValues[]  = {0F, 0F, 0F};

    Gamepad oldGamePad2 = new Gamepad();

    private final Telemetry.Item intakeTelem;
    private final Telemetry.Item colorTelem;

    private final ElapsedTimer intakeLoopTimer = new ElapsedTimer();

    private final Telemetry.Item intakeLoopTime;

    private final Telemetry.Item servoBusCurrentTelem;

    private final ElapsedTimer servoBusCurrentUpdateTimer = new ElapsedTimer();

    private final ReusableHardwareAction servoBusCurrentHardwareAction;

    private final ElapsedTimer breakBeamUpdateTimer = new ElapsedTimer();

    private final ReusableHardwareAction breakBeamUpdateHardwareAction;

    private final ReusableHardwareAction leftIntakeMotorHardwareAction, rightIntakeMotorHardwareAction, leftIntakeServoHardwareAction, rightIntakeServoHardwareAction, leftSpinnerServoHardwareAction, rightSpinnerServoHardwareAction;

    private int colorReads = 0;

    public NewIntake(SubSystemData data, Encoder horizontalSlideEncoder, TouchSensor breakBeam, Boolean blueAlliance, boolean teleOpControls, boolean init, DoubleSupplier getVoltage) {
        super(data);

        this.teleOpControls = teleOpControls;
        this.blueAlliance = blueAlliance;
        myRevHub = hardwareMap.get(LynxModule.class, "Expansion Hub 3");

        this.servoBusCurrentHardwareAction = new ReusableHardwareAction(hardwareQueue);
        this.breakBeamUpdateHardwareAction = new ReusableHardwareAction(hardwareQueue);

        this.leftIntakeMotorHardwareAction = new ReusableHardwareAction(hardwareQueue);
        this.rightIntakeMotorHardwareAction = new ReusableHardwareAction(hardwareQueue);
        this.leftIntakeServoHardwareAction = new ReusableHardwareAction(hardwareQueue);
        this.rightIntakeServoHardwareAction = new ReusableHardwareAction(hardwareQueue);
        this.leftSpinnerServoHardwareAction = new ReusableHardwareAction(hardwareQueue);
        this.rightSpinnerServoHardwareAction = new ReusableHardwareAction(hardwareQueue);


        //Motors
        this.horizontalSlideEncoder = horizontalSlideEncoder;
        horizontalSlideEncoder.setDirection(Encoder.Direction.REVERSE);

        this.breakBeam = breakBeam;

        horizontalLeftMotor = hardwareMap.get(DcMotorEx.class, "horizontalLeft"); // control hub 0
        horizontalRightMotor = hardwareMap.get(DcMotorEx.class, "horizontalRight"); // control hub 1

        horizontalLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        horizontalLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.getVoltage = getVoltage;


        //Servos
        leftIntakeServo = hardwareMap.get(Servo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(Servo.class, "rightIntakeServo");


        leftIntakeServo.setDirection(Servo.Direction.REVERSE);


        if (init) {
            leftIntakeServo.setPosition(targetIntakePos);
            rightIntakeServo.setPosition(targetIntakePos);
        }

        colors.red = 0;
        colors.green = 0;
        colors.blue = 0;

        newColors = colors;

        leftSpinnerServo = hardwareMap.get(CRServo.class, "leftSpinnerServo");
        rightSpinnerServo = hardwareMap.get(CRServo.class, "rightSpinnerServo");

        leftSpinnerServo.setDirection(DcMotorSimple.Direction.REVERSE);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        colorSensor.setGain(30);


        intakeTelem = telemetry.addData("Intake state", intakeState.name());
        colorTelem = telemetry.addData("Color Telem", "");
        //initiating slide encoder
        slidePos = ticksToInches(horizontalSlideEncoder.getCurrentPosition());

        servoBusCurrentTelem = telemetry.addData("Servo Bus Current", "");

        if (PassData.horizontalSlidesInitiated && slidePos>-.05) {
            //If slides are not at the bottom they are set to their current pose
            if (slidePos>1) {
                targetSlidePos = slidePos;
            }
        } else {

            horizontalLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            slidePos = 0;
            targetSlidePos = 0;

            PassData.horizontalSlidesInitiated = true;
        }

        newTargetSlidePos = targetSlidePos;

        slidePosTelem = telemetry.addData("Slide position", "");

        intakeLoopTime = telemetry.addData("Intake loop time", "");

    }



    @Override
    public void priorityData() {
//        if (changedIntakeState) {
//            intakeState = newIntakeState;
//            changedIntakeState = false;
//        }
//
//        servoBusCurrent = getServoBusCurrent();
        if (servoBusCurrent != updatedServoBusCurrent) {
            changedServoBusCurrent = true;

            servoBusCurrent = updatedServoBusCurrent;
        } else {
            changedServoBusCurrent = false;
        }

        slideTicks = horizontalSlideEncoder.getCurrentPosition();

        if (changedIsBreakBeam) {
            prevIsBreakBeam = isBreakBeam;
            isBreakBeam = updatedIsBreakBeam;

            changedIsBreakBeam = false;
        }

//        isBreakBeam = breakBeam.isPressed();

        if (newColors != null && checkColor) {
            colors = newColors;
        }

//        checkColor = nextCheckColor;

        if (!checkColor && (((isBreakBeam) && intakingState == IntakingState.INTAKING) || (teleOpControls && isBreakBeam && gamepad2.left_bumper && !oldGamePad2.left_bumper))) {
            colorReads = 0;
            checkColor = true;
            if (blueAlliance != null) {
                colors = colorSensor.getNormalizedColors();
            }
        }

        if (updateIntakeState) {
            intakeState = newIntakeState;

            updateIntakeState = false;
        }

        if (updateIntakingState) {
            intakingState = newIntakingState;

            updateIntakingState = false;
        }

        if (changedToIntakeState) {
            toIntakeState = newToIntakeState;
            changedToIntakeState = false;
        }

        if (changedTargetSlidePos) {
            targetSlidePos = newTargetSlidePos;
            changedTargetSlidePos = false;
        }

        voltage = getVoltage.getAsDouble();
        prevIntakingState = intakingState;

    }

    //trying to add to things to hardware queue asap so their more time for it to be called
    @Override
    public void loop() {
        intakeLoopTimer.reset();

        if (teleOpControls) {
            if (gamepad2.back) {
                if (gamepad2.dpad_down) {
                    horizontalLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    targetSlidePos = 0;
                } else if (gamepad2.dpad_up) {
                    intakingState = IntakingState.START_UNJAMMING;
                }

                if (Math.abs(gamepad2.left_stick_y)>.05) {
                    targetSlidePos = targetSlidePos + 10 * slideTimer.seconds() * -gamepad2.left_stick_y;
                }

                if (gamepad2.start) {
                    leftSpinnerServo.getController().pwmDisable();
                    rightSpinnerServo.getController().pwmDisable();
                    leftIntakeServo.getController().pwmDisable();
                    rightIntakeServo.getController().pwmDisable();

                }

                if (gamepad2.x && !oldGamePad2.x) {
                    targetSlidePos = .5;
                }

            } else {
                if (gamepad2.dpad_down && !oldGamePad2.dpad_down) {
                    toIntakeState = ToIntakeState.RETRACT;
                    if (intakingState != IntakingState.FINISH_INTAKING) {
                        intakingState = IntakingState.IDLE;
                        targetIntakeSpeed = 0;
                    }
                } else if (gamepad2.dpad_right && !oldGamePad2.dpad_right) {
                    targetSlidePos = HorizontalSlide.CLOSE.length;
                } else if (gamepad2.dpad_left && !oldGamePad2.dpad_left) {
                    targetSlidePos = HorizontalSlide.MEDIUM.length;
                } else if (gamepad2.dpad_up && !oldGamePad2.dpad_up) {
                    targetSlidePos = HorizontalSlide.FAR.length;
                } else if (Math.abs(gamepad2.left_stick_y) > .05) {
                    if (!gamepad2.start) {
                        targetSlidePos = MathUtil.clip(targetSlidePos + 10 * slideTimer.seconds() * -gamepad2.left_stick_y, -.5, 18.5);
    //                    intake.setTargetSlidePos(intake.getTargetSlidePos() + 10 * loopTimer.seconds() * -gamepad2.left_stick_y * (1 - gamepad2.right_trigger * .75));
                    } else {
    //                    intake.setTargetIntakePos(intake.getTargetIntakePos() + gamepad2.left_stick_y * .0002 * loopTimer.milliSeconds());
                    }
                }
             }

            if (gamepad2.left_bumper && !oldGamePad2.left_bumper) {
                toIntakeState = ToIntakeState.DROP_INTAKE;
                intakingState = IntakingState.INTAKING;
                targetIntakeSpeed = 1;
            } else if (gamepad2.left_trigger>.2 && oldGamePad2.left_trigger<=.2) {
                if (targetIntakePos == IntakePos.UP.pos) {
                    targetIntakePos = IntakePos.AUTO_HEIGHT.pos;
                }
                targetIntakeSpeed = -1;
                intakingState = IntakingState.MANUAL_EJECTING;
            } else if ((!gamepad2.left_bumper && oldGamePad2.left_bumper) || (oldGamePad2.left_trigger>.2 && gamepad2.left_trigger<=.2)) {
                if (intakingState == IntakingState.INTAKING || intakingState == IntakingState.MANUAL_EJECTING) {
                    targetIntakeSpeed = 0;
                    intakingState = IntakingState.IDLE;
                    checkColor = false;
                    if (targetIntakePos == IntakePos.AUTO_HEIGHT.pos) {
                        toIntakeState(ToIntakeState.RETRACT_AND_STOP_INTAKING);
                    }
                }
            }

        }

        switch (toIntakeState) {
            case EXTEND_THEN_DROP_INTAKE:
                intakeState = IntakeState.EXTENDING;

                toIntakeState = ToIntakeState.IDLE;
                break;
            case DROP_INTAKE:
                targetIntakePos = IntakePos.DOWN.pos;

                intakeState = IntakeState.INTAKING;

                toIntakeState = ToIntakeState.IDLE;
                break;
            case DROP_INTAKE_AUTO_SHOVE_HEIGHT:
                targetIntakePos = IntakePos.AUTO_SHOVE_HEIGHT.pos;

                toIntakeState = ToIntakeState.IDLE;
                break;
            case PARTIAL_RAISE_INTAKE:
                targetIntakePos = IntakePos.PARTIAL_UP.pos;

                toIntakeState = ToIntakeState.IDLE;
                break;
            case RAISE_TO_AUTO_HEIGHT:
                targetIntakePos = IntakePos.AUTO_HEIGHT.pos;

                toIntakeState = ToIntakeState.IDLE;
                break;
            case RAISE_INTAKE:
                targetIntakePos = IntakePos.UP.pos;

                toIntakeState = ToIntakeState.IDLE;
                break;
            case SEARCH_POSITION:
                targetIntakePos = IntakePos.SEARCH.pos;
                targetSlidePos = 3;
                targetIntakeSpeed = 0;

                toIntakeState = ToIntakeState.IDLE;
                break;
            case RETRACT:
                targetIntakePos = IntakePos.UP.pos;
                intakeTimer.reset();

                intakeState = IntakeState.RETRACTING_INTAKE;

                toIntakeState = ToIntakeState.IDLE;
                break;
            case RETRACT_AND_STOP_INTAKING:
                targetIntakeSpeed = 0;
                intakingState = IntakingState.IDLE;

                targetIntakePos = IntakePos.UP.pos;
                intakeTimer.reset();

                intakeState = IntakeState.RETRACTING_INTAKE;

                toIntakeState = ToIntakeState.IDLE;
                break;


        }

        if (breakBeamUpdateTimer.milliSeconds()>20) {
            breakBeamUpdateHardwareAction.setAndQueueIfEmpty(() -> {
                updatedIsBreakBeam = breakBeam.isPressed();
                changedIsBreakBeam = true;
            });
            breakBeamUpdateTimer.reset();
        }

        if (servoBusCurrentUpdateTimer.milliSeconds() > 100 && targetIntakeSpeed > 0) {
            servoBusCurrentHardwareAction.setAndQueueIfEmpty(() -> {
                updatedServoBusCurrent = getServoBusCurrent();
            });

            servoBusCurrentUpdateTimer.reset();
        }


        //slide PID
        slidePos = ticksToInches(slideTicks);
        //limits max time
        double elapsedTime = Math.min(slideTimer.seconds(), .5);

        //pid control
        double error = targetSlidePos - slidePos;
        double absError = Math.abs(error);

        double p, d = 0, f = 0;

//        slideI += error*elapsedTime;

        //Checks if error is in acceptable amounts
        if (absError<.1) {
            p = 0;
        } else if (absError>3) {
            //Slides set to max power
            p = Math.signum(error);
        } else {//if (error<4 but error>.1)
            p = error*.28;//.35;
            d = ((prevSlideError-error) / elapsedTime) * .011;//.03;//.007
            f=Math.signum(error)*0.07;//.15;
        }

//        p = 0;
//        f=.3*Math.signum(error);

        double motorPower = (p - d) + f;//*Math.abs(p - d)
        slideTimer.reset();
        prevSlideError = error;


        if ((actualMotorPower == 0 && motorPower != 0) || (actualMotorPower != 0 && motorPower == 0) || (Math.abs(motorPower-actualMotorPower)>.05)) {
            leftIntakeMotorHardwareAction.setAndQueueAction(() -> horizontalLeftMotor.setPower(motorPower));// * 12/voltage
            rightIntakeMotorHardwareAction.setAndQueueAction(() -> horizontalRightMotor.setPower(motorPower));// * 12/voltage

            actualMotorPower = motorPower;
        }



        if (targetIntakePos != actualIntakePos) {
            leftIntakeServoHardwareAction.setAndQueueAction(() -> leftIntakeServo.setPosition(targetIntakePos+.03));
            rightIntakeServoHardwareAction.setAndQueueAction(() -> rightIntakeServo.setPosition(targetIntakePos + .01));

            actualIntakePos = targetIntakePos;
        }

        if (Math.abs(targetIntakeSpeed-actualIntakeSpeed)>.04) {
            leftSpinnerServoHardwareAction.setAndQueueAction(() -> leftSpinnerServo.setPower(targetIntakeSpeed));
            rightSpinnerServoHardwareAction.setAndQueueAction(() -> rightSpinnerServo.setPower(targetIntakeSpeed));

            actualIntakeSpeed = targetIntakeSpeed;
        }
        if (servoBusCurrent < 4) {
            servoStallTimer.reset();
        }

        switch (intakingState) {
            case START_INTAKING:
                targetIntakeSpeed = 1;
                intakingState = IntakingState.INTAKING;
                break;
            case INTAKING:

                if (changedServoBusCurrent && servoStallTimer.seconds() > .3) {
                    intakingState = IntakingState.SERVO_STALL_START_UNJAMMING;
                } else if (checkColor) {

                    sampleColor = findSampleColor();

                    if (blueAlliance == null || (sampleColor == SampleColor.NONE && colorReads > 10)) {
//                        throw new RuntimeException(String.format("Not nothing RGB: %.4f, %.4f, %.4f, HSV: %.4f, %.4f, %.4f", colors.red, colors.green, colors.blue, hsvValues[0], hsvValues[1], hsvValues[2]));

                        targetIntakePos = IntakePos.UP.pos;

                        intakeState = IntakeState.RETRACTING_INTAKE;

//                        if (!teleOpControls) {
                            intakingState = IntakingState.INTAKING_A_LITTLE_MORE;
//                        } else {
//                            intakingState = IntakingState.FINISH_INTAKING;
//                        }

                        intakingTimer.reset();
                        intakeTimer.reset();

                        checkColor = false;

                        break;
                    }

                    if (sampleColor == SampleColor.NONE) {
                        hardwareQueue.add(() -> {
                            newColors = colorSensor.getNormalizedColors();
                        });
                        colorReads++;
                        break;
                    }

//                    if (sampleColor == SampleColor.YELLOW) {
//                        throw new RuntimeException(String.format("Not yellow RGB: %.4f, %.4f, %.4f, HSV: %.4f, %.4f, %.4f", colors.red, colors.green, colors.blue, hsvValues[0], hsvValues[1], hsvValues[2]));
//                    } else if (sampleColor == SampleColor.RED) {
//                        throw new RuntimeException(String.format("Not red RGB: %.4f, %.4f, %.4f, HSV: %.4f, %.4f, %.4f", colors.red, colors.green, colors.blue, hsvValues[0], hsvValues[1], hsvValues[2]));
//                    }

                    checkColor = false;

                    if ((sampleColor == SampleColor.BLUE && !blueAlliance) ||  (sampleColor == SampleColor.RED && blueAlliance)) {//(
                        targetIntakeSpeed = -1;
                        targetIntakePos = IntakePos.PARTIAL_UP.pos;
                        intakingState = IntakingState.START_EJECTING;
                        intakingTimer.reset();
                    } else {
                        targetIntakePos = IntakePos.UP.pos;

                        intakeState = IntakeState.RETRACTING_INTAKE;

//                        if (!teleOpControls) {
                            intakingState = IntakingState.INTAKING_A_LITTLE_MORE;
//                        } else {
//                            intakingState = IntakingState.FINISH_INTAKING;
//                        }

                        intakingTimer.reset();
                        intakeTimer.reset();
                    }

                }
                break;
            case INTAKING_A_LITTLE_MORE:
                if (intakingTimer.seconds()>.1) {
                    leftSpinnerServoHardwareAction.setAndQueueAction(() -> {
                        leftSpinnerServo.setPower(-1);
                    });
                    intakingTimer.reset();
                    intakingState = IntakingState.INTAKING_SPIN_OUT;
                }
                break;
            case INTAKING_SPIN_OUT:
                if (intakingTimer.seconds()>.3) {
                    leftSpinnerServoHardwareAction.setAndQueueAction(() -> {
                        leftSpinnerServo.setPower(1);
                    });
                    intakingTimer.reset();
                    intakingState = IntakingState.INTAKING_IN_AGAIN;
                }
                break;
            case INTAKING_IN_AGAIN:
                if (intakingTimer.seconds()>.2) {
                    intakingTimer.reset();
                    intakingState = IntakingState.FINISH_INTAKING;
                }
                break;
            case FINISH_INTAKING:
                if (changedServoBusCurrent && servoStallTimer.seconds() > .3) {
                    intakingState = IntakingState.SERVO_STALL_START_UNJAMMING;
                } if (intakingTimer.seconds()>.8 || targetIntakeSpeed == 0) {
//                    targetIntakeSpeed = 0;

                    intakingState = IntakingState.HOLDING_SAMPLE;
                }
                break;
            case START_EJECTING:
                if (!isBreakBeam || intakingTimer.seconds()>1) {
                    intakingTimer.reset();

                    intakingState = IntakingState.FINISH_EJECTING;
                }
                break;
            case FINISH_EJECTING:
                if (intakingTimer.seconds()>1) {
                    targetIntakeSpeed = 0;
                    targetIntakePos = IntakePos.DOWN.pos;
                    intakingState = IntakingState.IDLE;
                }
                break;
            case MANUAL_EJECTING:

                break;


            case START_REINTAKING:
                targetIntakeSpeed = 1;
                intakingTimer.reset();

                intakingState = IntakingState.FINISH_INTAKING;
                break;
            case FINISH_REINTAKING:
                if (intakingTimer.seconds()>1.5) {
                    targetIntakeSpeed = 0;

                    intakingState = IntakingState.HOLDING_SAMPLE;
                }
                break;


            case START_UNJAMMING:
                intakingTimer.reset();
                targetIntakeSpeed = -1;
                intakingState = IntakingState.UNJAMMING_SPIN_OUT;
                break;
            case UNJAMMING_SPIN_OUT:
                if (intakingTimer.seconds()>.1) {
                    targetIntakeSpeed = 1;
                    intakingState = IntakingState.UNJAMMING_SPIN_IN;
                    intakingTimer.reset();
                }
                break;
            case UNJAMMING_SPIN_IN:
                if (isBreakBeam || intakingTimer.seconds() > .5) {
                    intakingTimer.reset();
                    intakingState = IntakingState.UNJAMMING_FINISH_SPIN_IN;
                }
                break;
            case UNJAMMING_FINISH_SPIN_IN:
                if (intakingTimer.seconds()>1.5) {
                    targetIntakeSpeed = 0;
                    intakingState = IntakingState.HOLDING_SAMPLE;
                }
                break;
            case SERVO_STALL_START_UNJAMMING:
                targetIntakeSpeed = -1;
                intakingState = IntakingState.SERVO_STALL_UNJAMMING_SPIN_OUT;
                if (timeSinceLastStall.seconds()>.8) {
                   stallCount = 0;
                }
                timeSinceLastStall.reset();

                intakingTimer.reset();
                break;
            case SERVO_STALL_UNJAMMING_SPIN_OUT:
                if ((stallCount == 0 && intakingTimer.seconds()>.05) || (stallCount == 1 && intakingTimer.seconds()>.1) || (intakingTimer.seconds()>.2)) {
                    if (!teleOpControls || gamepad2.left_bumper) {
                        targetIntakeSpeed = 1;

                    }
                    else {
                        targetIntakeSpeed = 0;
                    }
                    intakingState = IntakingState.INTAKING;
                    intakingTimer.reset();
                    stallCount++;
                }
                break;


            case START_PARTIAL_GRAB:
                targetIntakeSpeed = .3;
                intakingState = IntakingState.INTAKING_PARTIAL_GRAB;
                intakingTimer.reset();
                break;
            case INTAKING_PARTIAL_GRAB:
                if (isBreakBeam || intakingTimer.seconds()>1) {
                    targetIntakeSpeed = 0;
                    intakingState = IntakingState.IDLE;
                }
                break;

            case START_EJECTING_PARTIAL_GRAB:
                targetIntakeSpeed = -1;
                targetIntakePos = IntakePos.PARTIAL_UP.pos;
                intakingState = IntakingState.INTAKING_PARTIAL_GRAB;
                intakingTimer.reset();
                break;
            case EJECTING_PARTIAL_GRAB:
                if (intakingTimer.seconds()>.3) {
                    targetIntakeSpeed = 0;
                    intakingState = IntakingState.IDLE;
                }
                break;


        }

        if (teleOpControls && gamepad2.right_trigger > .05 && (intakeState == IntakeState.INTAKING || intakeState == IntakeState.RESTING || intakeState == IntakeState.DROPPING_INTAKE || intakeState == IntakeState.EXTENDING)) {
            targetIntakePos = IntakePos.DOWN.pos+gamepad2.right_trigger*(IntakePos.UP.pos-IntakePos.DOWN.pos);
        } else if (gamepad2.right_trigger <= .05 && oldGamePad2.right_trigger > .05) {
            targetIntakePos = IntakePos.DOWN.pos;
        }


        switch (intakeState) {
            case EXTENDING:
                //if slides past bar or need to be dropped
                //add more logic
                if (slidePos>2) {
                    targetIntakePos = IntakePos.DOWN.pos;
//                    intakeState = IntakeState.INTAKING;
                    intakeTimer.reset();

                    intakeState = IntakeState.DROPPING_INTAKE;
                }
                break;
            case DROPPING_INTAKE:
                //if intake has finished dropping start intaking
                if (intakeTimer.seconds()>.2) {
                    targetIntakeSpeed = 1;



                    intakeState = IntakeState.INTAKING;
                }
                break;

            case INTAKING:
//                if (holdingSample) {
//                    targetIntakeSpeed = .1;
//
//                    targetIntakePos = IntakePos.UP.pos;
//
////                    targetSlidePos = HorizontalSlide.IN.length;
//
//                    intakeState = IntakeState.RETRACTING_INTAKE;
//
//                    intakeTimer.reset();
//                }
                break;
            case RETRACTING_INTAKE:
                if (intakeTimer.seconds()>.3) {
                    targetSlidePos = HorizontalSlide.EXTRA_IN.length;

                    intakeState = IntakeState.RETRACTING;
                }
                break;
            case RETRACTING:
                //if fully retracted and holding sample start transferring else rest
                if ((slidePos<.2 && intakingTimer.seconds() > .2)  || intakeTimer.seconds()>1) {
                    intakeTimer.reset();

                    intakeState = IntakeState.WAITING_AFTER_RETRACTING;
                }
                break;
            case WAITING_AFTER_RETRACTING:
                if (intakeTimer.seconds()>.1) {
                    targetSlidePos = HorizontalSlide.IN.length;

                    if (intakingState == IntakingState.HOLDING_SAMPLE || (isBreakBeam && (intakingState == IntakingState.IDLE || intakingState == IntakingState.FINISH_INTAKING))) {
//                        targetIntakeSpeed = 0;
                        if (isBreakBeam) {
                            transfer = true;
                            intakeState = IntakeState.WAITING_FOR_TRANSFERRING;
                        } else {
                            intakeState = IntakeState.WAITING_FOR_TRANSFER;
                            intakingState = IntakingState.START_UNJAMMING;
                        }

                    } else {
                        intakeState = IntakeState.WAITING_FOR_TRANSFER;
                    }
                }
                break;
            case WAITING_FOR_TRANSFER:
                if (intakingState == IntakingState.HOLDING_SAMPLE || intakingState == IntakingState.FINISH_INTAKING || intakingState == IntakingState.UNJAMMING_FINISH_SPIN_IN) {
                    if (isBreakBeam) {
                        transfer = true;
                        intakeState = IntakeState.WAITING_FOR_TRANSFERRING;
                    } else {
                        intakingState = IntakingState.START_UNJAMMING;
                    }
                }
                break;
            case WAITING_FOR_TRANSFERRING:
                //if transferred go to resting position, add more logic later
                if (!transfer) {
                    intakeState = IntakeState.TRANSFERRING;
                    intakeTimer.reset();
                }
                break;
            case TRANSFERRING:
                if (intakeTimer.seconds() > .5) {
                    targetIntakeSpeed = 0;
                    intakingState = IntakingState.IDLE;
                    intakeState = IntakeState.RESTING;

                }
                break;
            case MOVE_SLIDES_MORE_IN:
                setTargetSlidePos(-2.5);
                intakeState =  IntakeState.RESET_SLIDES;
                intakeTimer.reset();
                break;
            case RESET_SLIDES:
                if (intakeTimer.seconds() > .5) {
                    horizontalLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    intakeState = IntakeState.WAITING_FOR_TRANSFER;
                }
                break;
        }

        intakeTelem.setValue(intakeState.name() + " Intaking state: " + intakingState.name());
        colorTelem.setValue(checkColor + " cur:" + isBreakBeam + " prev:" + prevIsBreakBeam);

        servoBusCurrentTelem.setValue(servoBusCurrent);
        prevIsBreakBeam = isBreakBeam;
        oldGamePad2.copy(gamepad2);

        intakeLoopTime.setValue(intakeLoopTimer.milliSeconds());
    }

    @SuppressLint("DefaultLocale")
    @Override
    public TelemetryPacket dashboard(TelemetryPacket packet) {

        slidePosTelem.setValue(String.format("%.2f Target: %.2f", slidePos, targetSlidePos));

        return super.dashboard(packet);
    }

    private SampleColor findSampleColor() {
                Color.colorToHSV(colors.toColor(), hsvValues);

//                return SampleColor.BLUE;
        if (hsvValues[2] < .13) {
            return SampleColor.NONE;
        } else if ((hsvValues[0] > 40 && hsvValues[0] < 80)) {
//            throw new RuntimeException("Not yellow");
            return SampleColor.YELLOW;
        } else if ((hsvValues[0] < 30)) {
//            throw new RuntimeException("Not red");
            return SampleColor.RED;
        } else if ((hsvValues[0] > 200 && hsvValues[0] < 240)) {
            return SampleColor.BLUE;
        } else {
            return SampleColor.NONE;
        }
    }

    public boolean holdingSample() {
        return breakBeam.isPressed();
    }

    private double ticksToInches(int ticks) {
        return (ticks/145.1)*4.72;
    }


    //only call during priority data to avoid threading issues
    public boolean transfer() {
        if (transfer) {
            transfer = false;
            return true;
        } else {
            return false;
        }
    }

    public SampleColor getSampleColor() {
        return sampleColor;
    }


    //single thread safe
    public void setIntakingState(IntakingState intakingState) {
        newIntakingState = intakingState;
        updateIntakingState = true;
    }

    public void setIntakeState(IntakeState intakeState) {
        newIntakeState = intakeState;
        updateIntakeState = true;
    }

    public void toIntakeState(ToIntakeState toIntakeState) {
        newToIntakeState = toIntakeState;
        changedToIntakeState = true;
    }

    public void setTargetSlidePos(double distance) {
        newTargetSlidePos = distance;
        changedTargetSlidePos = true;
    }

    public void checkSlidesIn() {
        double currentSlidePos = slidePos;



    }

    public boolean isBreakBeam() {
        return isBreakBeam;
    }

    public IntakingState getPrevIntakingState() {
        return prevIntakingState;
    }

    public double getServoBusCurrent()
    {

        try {
            LynxGetADCCommand.Channel servoChannel = LynxGetADCCommand.Channel.SERVO_CURRENT;
            LynxGetADCCommand servoCommand = new LynxGetADCCommand(myRevHub, servoChannel, LynxGetADCCommand.Mode.ENGINEERING);

            return servoCommand.sendReceive().getValue() / 1000.0;
        } catch (InterruptedException | RuntimeException | LynxNackException e) {

        }
        return 0;
    }

    public double getActualSlidePos() {
        return slidePos;
    }

    public double getActualIntakePos() {
        return actualIntakePos;
    }

    public double getIntakeHorizontalOffset() {
        if (getActualIntakePos() == NewIntake.IntakePos.UP.pos) {
            return -.5;
        } else if (getActualIntakePos() == NewIntake.IntakePos.SEARCH.pos) {
            //TODO: measure these vals
            return  2.5;
        } else {// if (intake.getActualIntakePos() == NewIntake.IntakePos.DOWN.pos) {
            return 2.375;
        }
    }

    public double getIntakeVerticalOffset() {
        if (getActualIntakePos() == NewIntake.IntakePos.UP.pos) {
            return 8.5;
        } else if (getActualIntakePos() == NewIntake.IntakePos.SEARCH.pos) {
            return 7.875;
        } else {
            return 4.125;
        }
    }

    public IntakeState getPrevIntakeState() {
        return intakeState;
    }

    public void overrideSpinOut() {
        leftSpinnerServo.setPower(-1);
        rightSpinnerServo.setPower(-1);
    }

    public void stopMotors() {
        horizontalLeftMotor.setPower(0);
        horizontalRightMotor.setPower(0);
    }
}
