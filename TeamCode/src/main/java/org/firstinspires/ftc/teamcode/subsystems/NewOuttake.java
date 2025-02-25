package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.reefsharklibrary.misc.ElapsedTimer;
import com.reefsharklibrary.robotControl.ReusableHardwareAction;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.teamcode.PassData;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

public class NewOuttake extends SubSystem {

    public enum OuttakeState {
        EXTENDING_PLACE_BEHIND,
        EXTENDING_V4BAR_PLACE_BEHIND,
        WAITING_PLACE_BEHIND,
        PLACING_BEHIND,
        WAITING_CLEAR_BUCKET,
        RETRACTING_FROM_PLACE_BEHIND,

        EXTENDING_TO_DROP_SAMPLE,
        WAITING_DROP_SAMPLE,
        DROPPING_SAMPLE,
        MOVING_TO_GRAB_SPECIMEN,
        WAITING_GRAB_SPECIMEN,
        GRABBING_SPECIMEN,
        REMOVING_SPECIMEN_FROM_WALL,
        EXTENDING_PLACE_FRONT,
        WAITING_PLACE_FRONT,
        PLACING_FRONT,
        MOVING_TO_CLEAR_FRONT_BAR,
        MOVING_DOWN_TO_RETRACT,
        MOVING_ARM_BACK,
        RETRACING_FROM_PLACE_FRONT_CLEAR_INTAKE,

        MOVING_TO_DROP_HANG_HOOKS,
        MOVING_TO_REDROP_HANG_HOOKS,
        DROPPING_HANG_HOOKS,
        MOVING_TO_HANG_POSITION,
        WAITING_TO_HANG,

        START_RETRACTING_FROM_BEHIND,
        RETRACTING_FROM_BEHIND,

        WAITING_FOR_TRANSFER,
        GRABBING_FROM_TRANSFER,
        EXTRACTING_FROM_TRANSFER,
        VERIFYING_EXTRACTION,


        MOVING_TO_EJECTION,
        EJECTING,
        INIT_POSITION,
        IDLE,

        PULLING_TO_FIRST_BAR,
        SLOW_BEFORE_FIRST_BAR,
        FIRST_BAR_WAIT,
        EXTEND_TO_SECOND_BAR,
        GRABBED_SECOND_BAR

    }

    private OuttakeState outtakeState = OuttakeState.INIT_POSITION;

    private OuttakeState prevOuttakeState = outtakeState;

    private final Telemetry.Item outtakeStateTelem;

//    private Localizer localizer;

    public enum ToOuttakeState {
        WAIT_PLACE_FRONT,
        PLACE_FRONT,
        FIRST_PLACE_FRONT,
        RETRACT_FROM_PLACE_BEHIND,
        WAIT_PLACE_BEHIND,
        PLACE_BEHIND,
        WAIT_DROP_BEHIND,
        INIT_POSITION,
        TOUCH_BAR,
        HANG,
        IDLE,
        POWER_OFF_OUTTAKE_ARM
    }

    private ToOuttakeState toOuttakeState = ToOuttakeState.INIT_POSITION;

    private ToOuttakeState newToOuttakeState = toOuttakeState;

    private boolean changedToOuttakeState = true;

    private boolean failedToTransfer = false;


    private final ElapsedTimer outtakeTimer = new ElapsedTimer();


    public enum VerticalSlide {
        EXTRA_DOWN(-.3),
        DOWN(0),
        TRANSFER(5.65),
        EXTRACT_FROM_TRANSFER(9),
        MIN_PASSTHROUGH_HEIGHT(10),
        SPECIMEN_PICKUP(3.55),
        CLEAR_SPECIMEN_BAR(6.6),
        SPECIMEN_BAR(8),
        PLACE_SPECIMEN_BAR(13.3),
        HANG_HEIGHT(21),
        LOW_BUCKET_HEIGHT(5.65),//3
        HIGH_BUCKET(19.5),
        PULL_TO_FIRST_BAR(.3),
        GRAB_FIRST_BAR(3),
        EXTEND_TO_SECOND_BAR(18);

        public final double length;
        VerticalSlide(double length) {this.length = length;}
    }

    private boolean cycleHigh = true;

    private final ElapsedTimer slideTimer = new ElapsedTimer();

    private double targetSlidePos, newTargetSlidePos;

    private double slidePos;

    private double prevSlideError;

    private int slideTicks = 0;

    private boolean slideProfile = false;

    private double targetMotionProfilePos = 0;

    private double slideVel = 0;

    private double prevPitch, prevYaw;

    private final ElapsedTimer pitchTimer = new ElapsedTimer();

    private final ElapsedTimer yawTimer = new ElapsedTimer();

    private final ElapsedTimer slideProfileTimer = new ElapsedTimer();

    private double startingHangHeading = 0;

    private boolean neverResetIntake = true;

    private final DoubleSupplier getVoltage;

    private double voltage = 13;

    public enum V4BarPos {
        PLACE_FRONT(.34 - .042),
        FIRST_FRONT(.335 - .042),
        CLEAR_FRONT_BAR(.29 - .042),
//        WAIT_FOR_TRANSFER(.35),
        RELEASE_HANG_HOOKS(.53 - .042),
        MID_POSITION_CUTOFF(.55 - .042),
        WAITING_FOR_HANG_DEPLOY(.363),//.42
        // hello brett my king
        INIT(.37),
        TRANSFER(.446), //.444
        GRAB_BACK(.63 - .042),
        WAIT_PLACE_BACK(.14 - .042),
        PLACE_BACK(.12 - .042),//.07
        HANG_POS(.23 - .042),
        IDLE_POSITION(.41 - .042),
        TOUCH_BAR(.339 - .042);

        public final double pos;

        V4BarPos(double pos) {
            this.pos = pos;
        }
    }

    private double targetV4BPos = V4BarPos.INIT.pos;

    private double actualV4BPos = targetV4BPos;//set to -1 so target will never == actual on first loop


    public enum ClawPitch {
        LESS_DOWN(.41 - .019),
        DOWN(.38 - .019),
        BACK(0.09 - .019),
        BACK_ANGLED_DOWN(.19 - .019),
        BACK2(1 - .019),
        TRANSFER(.416), //.435
        EXTRACT_FROM_TRANSFER(.32 - .019),
        FRONT_ANGLED_UP(.81 - .019),
        FRONT_ANGELED_DOWN(.49),

        FRONT(.6- .019);

        public final double pos;

        ClawPitch(double pos) {
            this.pos = pos;
        }
    }

    private double targetClawPitch = ClawPitch.FRONT_ANGLED_UP.pos;

    private double actualClawPitch = targetClawPitch;


    public enum ClawPosition {
        EXTRA_OPEN(.28),//.49),.28
        HANG_DEPLOY(.26),// , .26
        OPEN(.22),//.36), .22
        PARTIALOPEN(.12),// , .12
        CLOSED(.045);//.15), .045

//        EXTRA_OPEN(.6),
//        OPEN(.4),
//        CLOSED(.09);

        public final double pos;

        ClawPosition(double pos) {
            this.pos = pos;
        }
    }

    private ClawPosition clawPosition = ClawPosition.CLOSED;

    private ClawPosition newClawPosition = clawPosition;

    private boolean changedClawPosition = false;

    private boolean updateClawPosition = false;


    private int hookDropCount = 0;

    private boolean transfer = false;

//    private boolean updateTransfer = false;

    private final boolean teleOpControls;
    private final boolean autoExtendSlides, autoRetractSlides;


    private final DcMotorEx verticalLeftMotor,verticalRightMotor;
    private Encoder verticalSlideEncoder;

    private double actualMotorPower = 0;

    private boolean cycleSpecimen = false;

    private Gamepad oldGamePad2 = new Gamepad();

    private final NewIntake intake;

    private NewIntake.SampleColor sampleColor = NewIntake.SampleColor.NONE;

    public Boolean blueAlliance;

    private boolean intakeHoldingSample = false;
    private boolean updateIntakeHoldingSample = false;

    private double transferAttemptCounter = 0;

    private final double maxTransferAttempts = 2;

    private final Servo clawServo, clawPitchServo, leftOuttakeServo, rightOuttakeServo;

    private final ElapsedTimer outtakeLoopTimer = new ElapsedTimer();

    private final Telemetry.Item outtakeMotorPower;
    private final ReusableHardwareAction leftMotorReusableHardwareAction, rightMotorReusableHardwareAction, leftOuttakeServoReusableHardwareAction, rightOuttakeServoReusableHardwareAction, clawPitchServoReusableHardwareAction, clawServoReusableHardwareAction;

    public NewOuttake(SubSystemData data, NewIntake intake, Encoder verticalSlideEncoder, Boolean blueAlliance, boolean teleOpControls, boolean autoExtendSlides, boolean autoRetractSlides, boolean init, DoubleSupplier getVoltage) {
        super(data);
//        this.localizer = localizer;
        this.teleOpControls = teleOpControls;
        this.autoExtendSlides = autoExtendSlides;
        this.autoRetractSlides = autoRetractSlides;

        this.blueAlliance = blueAlliance;

        this.intake = intake;

        this.verticalSlideEncoder = verticalSlideEncoder;

        this.getVoltage = getVoltage;

        this.leftMotorReusableHardwareAction = new ReusableHardwareAction(hardwareQueue);
        this.rightMotorReusableHardwareAction = new ReusableHardwareAction(hardwareQueue);
        this.leftOuttakeServoReusableHardwareAction = new ReusableHardwareAction(hardwareQueue);
        this.rightOuttakeServoReusableHardwareAction = new ReusableHardwareAction(hardwareQueue);
        this.clawPitchServoReusableHardwareAction = new ReusableHardwareAction(hardwareQueue);
        this.clawServoReusableHardwareAction = new ReusableHardwareAction(hardwareQueue);


        //motors
        verticalLeftMotor = hardwareMap.get(DcMotorEx.class, "verticalLeft"); // control hub 2
        verticalRightMotor = hardwareMap.get(DcMotorEx.class, "verticalRight"); // control hub 3

        verticalLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalRightMotor.setDirection(DcMotorEx.Direction.REVERSE);

        //encoder
//        verticalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalLeft"));
//        verticalSlideEncoder.setDirection(Encoder.Direction.REVERSE);

        //servos
        clawServo = hardwareMap.get(Servo.class, "clawServo"); // ex hub 4

        clawPitchServo = hardwareMap.get(Servo.class, "clawPitchServo"); // control hub 1

        leftOuttakeServo = hardwareMap.get(Servo.class, "leftOuttakeServo"); // ex hub 5
        rightOuttakeServo = hardwareMap.get(Servo.class, "rightOuttakeServo"); // control hub 4

        rightOuttakeServo.setDirection(Servo.Direction.REVERSE);


        //initiating slide encoder
        slidePos = ticksToInches(verticalSlideEncoder.getCurrentPosition());

        if (PassData.horizontalSlidesInitiated && slidePos>-.05) {
            //If slides are not at the bottom they are set to their current pose
            if (slidePos>1) {
                targetSlidePos = slidePos;
            }
        } else {
            verticalLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//            verticalRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            verticalRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            slidePos = 0;
            targetSlidePos = 0;

            PassData.horizontalSlidesInitiated = true;
        }

        newTargetSlidePos = targetSlidePos;

        if (init) {
            leftOuttakeServo.setPosition(targetV4BPos);
            rightOuttakeServo.setPosition(targetV4BPos);

            clawPitchServo.setPosition(targetClawPitch);

            clawServo.setPosition(ClawPosition.CLOSED.pos);//clawPosition.pos);

        }

        outtakeMotorPower = telemetry.addData("Outtake motor power", "");

        outtakeStateTelem = telemetry.addData("Outtakestate", outtakeState.toString());
    }


    @Override
    public void priorityData() {
        slideTicks = verticalSlideEncoder.getCurrentPosition();

        if (intake.transfer()) {
            transfer = true;
        }

        if (transfer && blueAlliance != null) {
            sampleColor = intake.getSampleColor();
//            transfer = true;
//            updateTransfer = false;
        }

        if (updateIntakeHoldingSample) {
            intakeHoldingSample = intake.holdingSample();
            updateIntakeHoldingSample = false;
        }

        if (changedToOuttakeState) {
            toOuttakeState = newToOuttakeState;
            changedToOuttakeState = false;
        }

        if (changedClawPosition) {
            clawPosition = newClawPosition;
            updateClawPosition = true;
            changedClawPosition = false;
        }

        voltage = getVoltage.getAsDouble();
        prevOuttakeState = outtakeState;
    }

    @Override
    public void loop() {
        outtakeLoopTimer.reset();

        if (teleOpControls) {
            if (gamepad2.back) {
                if (gamepad2.y && !oldGamePad2.y) {
                    cycleSpecimen = false;
                } else if (gamepad2.b && !oldGamePad2.b) {
                    cycleSpecimen = true;
                }

                if (gamepad2.a && !oldGamePad2.a) {
                    verticalLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    targetSlidePos = 0;
                }

                if (gamepad2.x && !oldGamePad2.x) {
                    toOuttakeState = ToOuttakeState.HANG;
                }

                if (Math.abs(gamepad2.right_stick_y) > .05) {
                    targetSlidePos = targetSlidePos+8 * slideTimer.seconds() * -gamepad2.right_stick_y * (1 - gamepad2.left_trigger * .75);
                    slideVel = 8 * -gamepad2.right_stick_y * (1 - gamepad2.left_trigger * .75);
                }
                else if (Math.abs(oldGamePad2.right_stick_y) > .05 && gamepad2.right_stick_y <= .05){
                    slideVel = 0;
                }

                if (Math.abs(gamepad2.left_stick_x)>.05) {
                    targetV4BPos += gamepad2.left_stick_x * slideTimer.seconds() * .05;
                }

                if (Math.abs(gamepad2.right_stick_x) > .05) {
                    targetClawPitch += gamepad2.right_stick_x * slideTimer.seconds() * .05;
                }
                if (gamepad2.right_bumper && !oldGamePad2.right_bumper) {
                    if (outtakeState == OuttakeState.GRABBED_SECOND_BAR) {
                        targetSlidePos = 0;
                    }
                    else {
                        outtakeState = OuttakeState.PULLING_TO_FIRST_BAR;
                        toSlidePosConstantVel(VerticalSlide.PULL_TO_FIRST_BAR.length, 40);
//                        startingHangHeading = ((TwoDeadWheelLocalizer)localizer).angles.getYaw();
                    }

                }

                if (gamepad2.start && !oldGamePad2.start) {
                    leftOuttakeServo.getController().pwmDisable();
                    rightOuttakeServo.getController().pwmDisable();
                    clawServo.getController().pwmDisable();
                    clawPitchServo.getController().pwmDisable();
//                    intake.blueAlliance = null;
//                    blueAlliance = null;
                }

            } else {

                if (gamepad2.a && !oldGamePad2.a) {
                    if (targetV4BPos<V4BarPos.MID_POSITION_CUTOFF.pos) {
                        retractFromFront();
//                        toOuttakeState = ToOuttakeState.RETRACT_FROM_PLACE_BEHIND;
                    } else {
                        retractFromGrabBehind();

                    }
                    slideProfile = false;
                    slideVel = 0;
                } else if (gamepad2.b && !oldGamePad2.b) {
                    cycleHigh = !cycleHigh;

                    if (cycleHigh) {
                        if (targetSlidePos == VerticalSlide.LOW_BUCKET_HEIGHT.length) {
                            targetSlidePos = VerticalSlide.HIGH_BUCKET.length;
                        }
                    } else {
                        if (targetSlidePos == VerticalSlide.HIGH_BUCKET.length) {
                            targetSlidePos = VerticalSlide.LOW_BUCKET_HEIGHT.length;
                        }
                    }

                } else if (gamepad2.x && !oldGamePad2.x) {
                    if (clawPosition == ClawPosition.OPEN) {
                        clawPosition = ClawPosition.PARTIALOPEN;
                        updateClawPosition = true;
                    }
//                    if (clawPosition == ClawPosition.CLOSED) {
                        dropBehind();
                    slideProfile = false;
                    slideVel = 0;
//                    } else if (clawPosition == ClawPosition.OPEN) {
//                        clawPosition = ClawPosition.EXTRA_OPEN;
//                        updateClawPosition = true;

//                    }
                } else if (gamepad2.y && !oldGamePad2.y) {
                    toOuttakeState = ToOuttakeState.PLACE_BEHIND;
                    slideProfile = false;
                    slideVel = 0;
                } else if (Math.abs(gamepad2.right_stick_y) > .05) {
                    if (!gamepad2.back) {
                        targetSlidePos = MathUtil.clip(targetSlidePos+8 * slideTimer.seconds() * -gamepad2.right_stick_y * (1 - gamepad2.left_trigger * .75), -.5, 28.1);
                        slideProfile = false;
                        slideVel = 8 * -gamepad2.right_stick_y * (1 - gamepad2.left_trigger * .75);
                    }
                    else if (Math.abs(oldGamePad2.right_stick_y) > .05 && gamepad2.right_stick_y <= .05){
                        slideVel = 0;
                    }
                }
                if (gamepad2.right_bumper && !oldGamePad2.right_bumper) {
                    updateClawPosition = true;
                    clawPosition = clawPosition == ClawPosition.CLOSED ? ClawPosition.OPEN : ClawPosition.CLOSED;

//                clawPosition = clawPosition == ClawPosition.CLOSED ? (outtakeState == OuttakeState.WAITING_DROP_SAMPLE ? ClawPosition.EXTRA_OPEN : ClawPosition.OPEN) : ClawPosition.CLOSED;
                }
            }






        }


        switch (toOuttakeState) {
            case PLACE_BEHIND:
                extendPlaceBehind();

                toOuttakeState = ToOuttakeState.IDLE;
                break;
            case PLACE_FRONT:
                extendPlaceFront();

                toOuttakeState = ToOuttakeState.IDLE;
                break;
            case FIRST_PLACE_FRONT:
                targetSlidePos = VerticalSlide.PLACE_SPECIMEN_BAR.length;

                targetV4BPos = V4BarPos.FIRST_FRONT.pos;

                targetClawPitch = ClawPitch.LESS_DOWN.pos;

                outtakeState = OuttakeState.EXTENDING_PLACE_FRONT;

                toOuttakeState = ToOuttakeState.IDLE;
                break;
            case RETRACT_FROM_PLACE_BEHIND:
                retractFromFront();

                toOuttakeState = ToOuttakeState.IDLE;
                break;
            case WAIT_DROP_BEHIND:
                dropBehind();
                toOuttakeState = ToOuttakeState.IDLE;
                break;
            case TOUCH_BAR:
                targetV4BPos = V4BarPos.TOUCH_BAR.pos;
                targetClawPitch = ClawPitch.FRONT_ANGLED_UP.pos;
                targetSlidePos = 6.5;
                toOuttakeState = ToOuttakeState.IDLE;
                outtakeState = OuttakeState.IDLE;
                break;
            case HANG:
                targetSlidePos = VerticalSlide.HANG_HEIGHT.length;
                targetV4BPos = V4BarPos.WAITING_FOR_HANG_DEPLOY.pos;
                targetClawPitch = ClawPitch.BACK.pos;
                if (targetV4BPos < V4BarPos.MID_POSITION_CUTOFF.pos) {
                    clawPosition = ClawPosition.HANG_DEPLOY;

                }
                else {
                    clawPosition = ClawPosition.PARTIALOPEN;
                }
                updateClawPosition = true;

                outtakeState = OuttakeState.MOVING_TO_DROP_HANG_HOOKS;

                outtakeTimer.reset();

                toOuttakeState = ToOuttakeState.IDLE;

                hookDropCount = 0;
                break;
            case POWER_OFF_OUTTAKE_ARM:
                leftOuttakeServo.getController().pwmDisable();
                rightOuttakeServo.getController().pwmDisable();
                toOuttakeState = ToOuttakeState.IDLE;
                break;
        }

        if (slideProfile) {
            double elapsedTime = Math.min(slideProfileTimer.seconds(), .005);
            if (targetMotionProfilePos > targetSlidePos) {
                targetSlidePos = Math.min(targetMotionProfilePos, targetSlidePos + slideVel * elapsedTime);
            }
            else if (targetMotionProfilePos < targetSlidePos) {
                targetSlidePos = Math.max(targetMotionProfilePos, targetSlidePos + slideVel * elapsedTime);
            }
            else {
                slideVel = 0;
                slideProfile = false;
            }

        }
        slideProfileTimer.reset();
        //slide PID
        slidePos = ticksToInches(slideTicks);
        //limits max time
        double elapsedTime = Math.min(slideTimer.seconds(), .5);

        //pid control
        double error = targetSlidePos - slidePos;
        double absError = Math.abs(error);

        double p, d = 0;

        double f;

        if (targetSlidePos != 0 && !slideProfile) {
            f = .11;
        } else {
            f = 0;
        }

        //Checks if error is in acceptable amounts

        if (absError>1) {
            //Slides set to max power
            p = 2*Math.signum(error);
        } else {
            p =error*.38;
            if (!slideProfile) {
                d = (slideVel - (prevSlideError-error) / elapsedTime) * .012;
            }


        }

        double motorPower =  p  + d + f; //
        slideTimer.reset();
        prevSlideError = error;

        if ((actualMotorPower == 0 && motorPower != 0) || (actualMotorPower != 0 && motorPower == 1) || (Math.abs(motorPower-actualMotorPower)>.05)) {
            leftMotorReusableHardwareAction.setAndQueueAction(() -> verticalLeftMotor.setPower(motorPower));// * 12/voltage
            rightMotorReusableHardwareAction.setAndQueueAction(() -> {
                verticalRightMotor.setPower(motorPower);// * 12/voltage
                outtakeMotorPower.setValue(motorPower);
            }
            );

            actualMotorPower = motorPower;
        }


        if (targetClawPitch != actualClawPitch) {
            clawPitchServoReusableHardwareAction.setAndQueueAction(() -> clawPitchServo.setPosition(targetClawPitch));

            actualClawPitch = targetClawPitch;
        }

        if (targetV4BPos != actualV4BPos) {
            leftOuttakeServoReusableHardwareAction.setAndQueueAction(() -> leftOuttakeServo.setPosition(targetV4BPos));
            rightOuttakeServoReusableHardwareAction.setAndQueueAction(() -> rightOuttakeServo.setPosition(targetV4BPos));

            actualV4BPos = targetV4BPos;
        }

        if (updateClawPosition) {
            clawServoReusableHardwareAction.setAndQueueAction(() -> clawServo.setPosition(clawPosition.pos));

            updateClawPosition = false;
        }



        switch (outtakeState) {
            case EXTENDING_PLACE_BEHIND:
                if (teleOpControls) {
                    if (absError<1) {
                        outtakeTimer.reset();
                        outtakeState = OuttakeState.EXTENDING_V4BAR_PLACE_BEHIND;
                    }
                }
                else {
                    if (absError<3) {
                        outtakeTimer.reset();
                        outtakeState = OuttakeState.EXTENDING_V4BAR_PLACE_BEHIND;
                        targetV4BPos = V4BarPos.PLACE_BACK.pos;
                    }
                }

                break;
            case EXTENDING_V4BAR_PLACE_BEHIND:
                if (teleOpControls) {
                    if (outtakeTimer.seconds()>.1) {
                        outtakeState = OuttakeState.WAITING_PLACE_BEHIND;
                    }
                }
                else{
                if (outtakeTimer.seconds()>.3) {
                    outtakeState = OuttakeState.WAITING_PLACE_BEHIND;
                }
                }
                break;
            case WAITING_PLACE_BEHIND:
                //claw opened by other code calling method
                if (clawPosition == ClawPosition.OPEN && autoRetractSlides) {
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.PLACING_BEHIND;
                }
                break;
            case PLACING_BEHIND:
                if (outtakeTimer.seconds()>.1) {
                    targetV4BPos = V4BarPos.TRANSFER.pos;

                    outtakeTimer.reset();

                    outtakeState = OuttakeState.WAITING_CLEAR_BUCKET;
                }
                break;
            case WAITING_CLEAR_BUCKET:
                if (outtakeTimer.seconds()>.3) {
                    retractFromFront();
                }
                break;
            case RETRACTING_FROM_PLACE_BEHIND:
                if (outtakeTimer.seconds()>.3 && absError<.5) {
                    if (targetV4BPos != V4BarPos.TRANSFER.pos) {
                        targetV4BPos = V4BarPos.TRANSFER.pos;
                        outtakeTimer.reset();
                    }
                    else {
                        outtakeState = OuttakeState.WAITING_FOR_TRANSFER;
                    }
                }
                break;


            case RETRACING_FROM_PLACE_FRONT_CLEAR_INTAKE:
                if (outtakeTimer.seconds()>.2) {
                    targetClawPitch = ClawPitch.BACK_ANGLED_DOWN.pos;

                    outtakeTimer.reset();
                    outtakeState = OuttakeState.EXTENDING_TO_DROP_SAMPLE;
                }
                break;
            case EXTENDING_TO_DROP_SAMPLE:
                if (outtakeTimer.seconds()>.1) {
                    targetSlidePos = VerticalSlide.SPECIMEN_PICKUP.length;

                    if (clawPosition != ClawPosition.CLOSED) {
                        clawPosition = ClawPosition.EXTRA_OPEN;
                        updateClawPosition = true;
                        targetClawPitch = ClawPitch.BACK.pos;
                        outtakeState = OuttakeState.MOVING_TO_GRAB_SPECIMEN;
                    }
                    else {
                        outtakeState = OuttakeState.WAITING_DROP_SAMPLE;
                    }
                }
                break;
            case WAITING_DROP_SAMPLE:
                if (clawPosition == ClawPosition.OPEN || clawPosition == ClawPosition.EXTRA_OPEN) {
                    if (clawPosition != ClawPosition.EXTRA_OPEN) {
                        clawPosition = ClawPosition.EXTRA_OPEN;
                        updateClawPosition = true;
                    }

                    outtakeTimer.reset();

                    outtakeState = OuttakeState.DROPPING_SAMPLE;
                }
                 break;
            case DROPPING_SAMPLE:
                if (outtakeTimer.seconds()>.1) {
                    targetClawPitch = ClawPitch.BACK.pos;
                    outtakeTimer.reset();

                    outtakeState = OuttakeState.MOVING_TO_GRAB_SPECIMEN;
                }
                break;
            case MOVING_TO_GRAB_SPECIMEN:
                if (outtakeTimer.seconds()>0) {
                    outtakeState = OuttakeState.WAITING_GRAB_SPECIMEN;
                }
                break;
            case WAITING_GRAB_SPECIMEN:
                if (clawPosition == ClawPosition.CLOSED && autoRetractSlides) {
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.GRABBING_SPECIMEN;
                }
                break;
            case GRABBING_SPECIMEN:
                if (outtakeTimer.seconds()>.1) {
                    targetSlidePos = VerticalSlide.SPECIMEN_PICKUP.length+4;
//                    targetV4BPos = V4BarPos.EXTRACT_FROM_GRAB_BACK.pos;
//                    targetClawPitch = ClawPitch.EXTRACT_FROM_TRANSFER.pos;

                    outtakeTimer.reset();

                    outtakeState = OuttakeState.REMOVING_SPECIMEN_FROM_WALL;
                }
                break;

            case REMOVING_SPECIMEN_FROM_WALL:
                if (outtakeTimer.seconds()>.2) {
                    extendPlaceFront();
                }
                break;
//            case EXTENDING_CLEAR_TRANSFER:
//                if (slidePos>VerticalSlide.MIN_PASSTHROUGH_HEIGHT.length) {
//                    targetV4BPos = V4BarPos.PLACE_BACK.pos;
//
//                    outtakeTimer.reset();
//
//                    outtakeState = OuttakeState.EXTENDING_PLACE_BEHIND;
//                }
//                break;
            case EXTENDING_PLACE_FRONT:
                if (absError<.5) {
                    outtakeState = OuttakeState.WAITING_PLACE_FRONT;

                }
                break;
            case WAITING_PLACE_FRONT:
                if (clawPosition == ClawPosition.OPEN) {
                    outtakeTimer.reset();

                    outtakeState = OuttakeState.PLACING_FRONT;
                }
                break;
            case PLACING_FRONT:
                if (outtakeTimer.seconds()>.3) {
                    targetV4BPos = V4BarPos.CLEAR_FRONT_BAR.pos;
                    outtakeTimer.reset();

                    outtakeState = OuttakeState.MOVING_TO_CLEAR_FRONT_BAR;
                }
                break;
            case MOVING_TO_CLEAR_FRONT_BAR:
                if (outtakeTimer.seconds()>.4) {
                    dropBehind();
//                    clawPosition = ClawPosition.PARTIALOPEN;
//
//                    outtakeState = OuttakeState.MOVING_DOWN_TO_RETRACT;
                }
                break;
            case MOVING_DOWN_TO_RETRACT:
                if (absError<.5) {
                    targetV4BPos = V4BarPos.TRANSFER.pos;
                    outtakeTimer.reset();

                    outtakeState = OuttakeState.MOVING_ARM_BACK;
                }
                break;
            case MOVING_ARM_BACK:
                if (outtakeTimer.seconds()>.4) {
//                    retractFromFront();
                    dropBehind();
                }
                break;

            case START_RETRACTING_FROM_BEHIND:
                if (outtakeTimer.seconds()>.3) {
                    targetV4BPos = V4BarPos.TRANSFER.pos;

                    outtakeTimer.reset();

                    outtakeState = OuttakeState.RETRACTING_FROM_BEHIND;
                }
                break;
            case RETRACTING_FROM_BEHIND:
                if (outtakeTimer.seconds()>.3) {
                    clawPosition = ClawPosition.OPEN;
                    updateClawPosition = true;

                    targetSlidePos = VerticalSlide.TRANSFER.length;

                    outtakeTimer.reset();

                    outtakeState = OuttakeState.RETRACTING_FROM_PLACE_BEHIND;
                }
                break;

            case MOVING_TO_DROP_HANG_HOOKS:
                if (outtakeTimer.seconds()>.5) {
                    if (clawPosition != ClawPosition.HANG_DEPLOY) {
                        clawPosition = ClawPosition.HANG_DEPLOY;
                        updateClawPosition = true;
                        outtakeTimer.reset();
                    }
                    else {
                        targetV4BPos = V4BarPos.RELEASE_HANG_HOOKS.pos;
                        outtakeState = OuttakeState.DROPPING_HANG_HOOKS;
                        outtakeTimer.reset();
                    }


                }
                break;
            case MOVING_TO_REDROP_HANG_HOOKS:
                if (outtakeTimer.seconds()>.2) {
                    targetV4BPos = V4BarPos.RELEASE_HANG_HOOKS.pos;
                    outtakeState = OuttakeState.DROPPING_HANG_HOOKS;
                    outtakeTimer.reset();

                }
                break;
            case DROPPING_HANG_HOOKS:
                if (outtakeTimer.seconds()>.55) {
                    if (hookDropCount >= 1) {
                        targetV4BPos = V4BarPos.HANG_POS.pos;
                        targetClawPitch = ClawPitch.FRONT_ANGLED_UP.pos;
                        clawPosition = ClawPosition.OPEN;
                        updateClawPosition = true;

                        outtakeState = OuttakeState.MOVING_TO_HANG_POSITION;
                    } else {
                        targetV4BPos = V4BarPos.WAITING_FOR_HANG_DEPLOY.pos;

                        outtakeState = OuttakeState.MOVING_TO_REDROP_HANG_HOOKS;

                        outtakeTimer.reset();

                        hookDropCount++;
                    }
                }
                break;
            case MOVING_TO_HANG_POSITION:

                break;


            case WAITING_FOR_TRANSFER:
                if (transfer) {
                    transfer = false;
                    if (cycleSpecimen && (blueAlliance == null || sampleColor != NewIntake.SampleColor.YELLOW)) {
                        outtakeState = OuttakeState.IDLE;
                    }
                    else {
                    clawPosition = ClawPosition.CLOSED;
                    updateClawPosition = true;

                    outtakeTimer.reset();

                    outtakeState = OuttakeState.GRABBING_FROM_TRANSFER;
                    }
                }
                break;
            case GRABBING_FROM_TRANSFER:
                if (outtakeTimer.seconds()>.2) { //changed from .2
                    targetSlidePos = VerticalSlide.EXTRACT_FROM_TRANSFER.length;

                    outtakeTimer.reset();

                    outtakeState = OuttakeState.EXTRACTING_FROM_TRANSFER;
                }
                break;
            case EXTRACTING_FROM_TRANSFER:
                if (outtakeTimer.seconds()>.2) {
                    updateIntakeHoldingSample = true;
                    outtakeState = OuttakeState.VERIFYING_EXTRACTION;
                }
                break;
            case VERIFYING_EXTRACTION:
                    //trys to grab sample again if first grab fails
                    if (intakeHoldingSample) {//intakeHoldingSample
                        if (neverResetIntake ) {//transferAttemptCounter == 0
                            retractFromFront();


                            intake.setIntakeState(NewIntake.IntakeState.MOVE_SLIDES_MORE_IN);
                            neverResetIntake = false;

                            intake.setIntakingState(NewIntake.IntakingState.START_REINTAKING);





                            transferAttemptCounter++;
                        } else if (transferAttemptCounter < maxTransferAttempts) {

                            retractFromFront();
                            intake.setIntakingState(NewIntake.IntakingState.START_UNJAMMING);
                            intake.setIntakeState(NewIntake.IntakeState.WAITING_FOR_TRANSFER);

                            transferAttemptCounter++;
                        } else {
                            targetSlidePos = VerticalSlide.TRANSFER.length;

                            targetV4BPos = V4BarPos.TRANSFER.pos;

                            targetClawPitch = ClawPitch.TRANSFER.pos;

                            clawPosition = ClawPosition.OPEN;
                            updateClawPosition = true;

                            outtakeState = OuttakeState.WAITING_FOR_TRANSFER;
                            failedToTransfer = true;

                            transferAttemptCounter = 0;
                        }

                        break;
                    }

                    transferAttemptCounter = 0;

                    if ((blueAlliance != null && ((sampleColor == NewIntake.SampleColor.RED && blueAlliance) || (sampleColor == NewIntake.SampleColor.BLUE && !blueAlliance)))){// {//gamepad2.right_trigger>.4 && oldGamePad2.right_trigger<=.4
                        intake.setIntakingState(NewIntake.IntakingState.START_EJECTING);
//                        targetSlidePos = VerticalSlide.TRANSFER.length + 2;
//                        targetV4BPos = V4BarPos.EJECT_OUT_FRONT.pos;
//                        targetClawPitch = ClawPitch.FRONT_ANGELED_DOWN.pos;

                        outtakeTimer.reset();

                        outtakeState = OuttakeState.WAITING_FOR_TRANSFER;
                    } else

                     if (autoExtendSlides) {
//                        if ( cycleSpecimen && (blueAlliance == null || sampleColor != NewIntake.SampleColor.YELLOW)) {
//                            dropBehind();
//                        } else {
                            extendPlaceBehind();
//                        }
                    } else {
                        outtakeState = OuttakeState.IDLE;
                    }
                break;


            case MOVING_TO_EJECTION:
                if (outtakeTimer.seconds()>.3) {
                    clawPosition = ClawPosition.OPEN;
                    updateClawPosition = true;

                    outtakeTimer.reset();

                    outtakeState = OuttakeState.EJECTING;
                }
                break;
            case EJECTING:
                if (outtakeTimer.seconds()>.2) {
                    retractFromFront();
                }
                break;
            case PULLING_TO_FIRST_BAR:
                //3in from top
                if (slidePos<targetMotionProfilePos+3) {
                    outtakeState = OuttakeState.SLOW_BEFORE_FIRST_BAR;
                    targetSlidePos = slidePos;
                    toSlidePosConstantVel(VerticalSlide.PULL_TO_FIRST_BAR.length, 10);
                    outtakeTimer.reset();
                }
                break;
            case SLOW_BEFORE_FIRST_BAR:
                if (!slideProfile && absError < .75 ) {
                    outtakeState = OuttakeState.WAITING_TO_HANG;
                    outtakeTimer.reset();
                }
                break;
            case WAITING_TO_HANG:
//                double pitch1 = ((TwoDeadWheelLocalizer)localizer).angles.getPitch();
//                double yaw = ((TwoDeadWheelLocalizer)localizer).angles.getYaw();
//
//                if (outtakeTimer.seconds() > 1 && pitch1 > -9  && ((pitch1 - prevPitch) / pitchTimer.seconds()) > 1
//                && (Math.abs(yaw - prevYaw) / yawTimer.seconds()) < 1 &&
//                Math.abs(startingHangHeading-yaw)<2) {
//                    toSlidePosConstantVel(VerticalSlide.GRAB_FIRST_BAR.length, 10);
                    toSlidePosConstantVel(VerticalSlide.EXTEND_TO_SECOND_BAR.length, 40);
//                    outtakeState = OuttakeState.FIRST_BAR_WAIT;
//                }
                break;
            case FIRST_BAR_WAIT:

                break;
            case EXTEND_TO_SECOND_BAR:
//                double pitch = ((TwoDeadWheelLocalizer)localizer).angles.getPitch();
//                if ((pitch > -3 || (pitch > -6 && (((pitch - prevPitch) / pitchTimer.seconds()) > 1))) && absError < .7) {
//                    targetSlidePos = VerticalSlide.EXTEND_TO_SECOND_BAR.length - 2;
//                    outtakeState = OuttakeState.GRABBED_SECOND_BAR;
//                }
                break;

        }
//        imuAngles.setValue(((TwoDeadWheelLocalizer) localizer).angles.getPitch() + " " + (((TwoDeadWheelLocalizer) localizer).angles.getPitch() - prevPitch) / pitchTimer.seconds()
//        + " yaw: " + ((TwoDeadWheelLocalizer) localizer).angles.getYaw() + " " +
//                (((TwoDeadWheelLocalizer) localizer).angles.getYaw() - prevYaw) / yawTimer.seconds());
//        prevPitch = ((TwoDeadWheelLocalizer)localizer).angles.getPitch();
//        prevYaw = ((TwoDeadWheelLocalizer)localizer).angles.getYaw();
        pitchTimer.reset();
        yawTimer.reset();

        oldGamePad2.copy(gamepad2);


        outtakeStateTelem.setValue(outtakeState.toString());
    }

    @Override
    public TelemetryPacket dashboard(TelemetryPacket packet) {
        return packet;
    }

    private double ticksToInches(int ticks) {
        return (ticks/537.7)*4.72;
    }

    private void extendPlaceBehind() {
        if (cycleHigh) {
            targetSlidePos = VerticalSlide.HIGH_BUCKET.length;
        } else {
            targetSlidePos = VerticalSlide.LOW_BUCKET_HEIGHT.length;
        }

        if (targetV4BPos > V4BarPos.MID_POSITION_CUTOFF.pos) {
            if (clawPosition != ClawPosition.CLOSED){
                clawPosition = ClawPosition.PARTIALOPEN;
                updateClawPosition = true;
            }
        }
//        if (teleOpControls) {
            targetV4BPos = V4BarPos.PLACE_BACK.pos;
//        }
//        else {
//            targetV4BPos = V4BarPos.WAIT_PLACE_BACK.pos;
//        }

        targetClawPitch = ClawPitch.BACK2.pos;

        outtakeState = OuttakeState.EXTENDING_PLACE_BEHIND;
    }

    private void extendPlaceFront() {

        if (targetV4BPos < V4BarPos.MID_POSITION_CUTOFF.pos) {
            targetSlidePos = VerticalSlide.PLACE_SPECIMEN_BAR.length;

            targetV4BPos = V4BarPos.PLACE_FRONT.pos;

            targetClawPitch = ClawPitch.DOWN.pos;

            outtakeState = OuttakeState.EXTENDING_PLACE_FRONT;
        }
        else {
            if (clawPosition != ClawPosition.CLOSED) {
                clawPosition = ClawPosition.PARTIALOPEN;
                updateClawPosition = true;

            }
            targetSlidePos = VerticalSlide.PLACE_SPECIMEN_BAR.length;
            targetV4BPos = V4BarPos.PLACE_FRONT.pos;

            targetClawPitch = ClawPitch.DOWN.pos;

            outtakeState = OuttakeState.EXTENDING_PLACE_FRONT;
        }

    }

//    private void extend

    private void dropBehind() {
        targetV4BPos = V4BarPos.GRAB_BACK.pos;
        targetClawPitch = ClawPitch.FRONT.pos;
        if (clawPosition != ClawPosition.CLOSED){
            clawPosition = ClawPosition.PARTIALOPEN;
            updateClawPosition = true;

        }

        //set to this so V4B has room to rotate, set lower after v4b is clear
        targetSlidePos = VerticalSlide.MIN_PASSTHROUGH_HEIGHT.length;

        outtakeTimer.reset();

        outtakeState = OuttakeState.RETRACING_FROM_PLACE_FRONT_CLEAR_INTAKE;
    }

    private void retractFromFront() {
        if (slidePos > VerticalSlide.TRANSFER.length - 1) {
            targetV4BPos = V4BarPos.TRANSFER.pos;

        }
        targetSlidePos = VerticalSlide.TRANSFER.length;


        targetClawPitch = ClawPitch.TRANSFER.pos;

        if (clawPosition != ClawPosition.OPEN) {
            clawPosition = ClawPosition.OPEN;
            updateClawPosition = true;
        }

        outtakeState = OuttakeState.RETRACTING_FROM_PLACE_BEHIND;

        outtakeTimer.reset();
    }

    private void retractFromGrabBehind() {
        targetSlidePos = VerticalSlide.MIN_PASSTHROUGH_HEIGHT.length;

        targetClawPitch = ClawPitch.TRANSFER.pos;

        if (clawPosition != ClawPosition.CLOSED) {
            clawPosition = ClawPosition.CLOSED;
            updateClawPosition = true;
        }

        outtakeTimer.reset();

        outtakeState = OuttakeState.START_RETRACTING_FROM_BEHIND;
    }


    public void toOuttakeState(ToOuttakeState toOuttakeState) {
        newToOuttakeState = toOuttakeState;
        changedToOuttakeState = true;
    }

    public void toClawPosition(ClawPosition clawPosition) {
        newClawPosition = clawPosition;
        changedClawPosition = true;
    }

    public OuttakeState getOuttakeState() {
        return prevOuttakeState;
    }

    public boolean getFailedToTransfer() {
        if (failedToTransfer){
            failedToTransfer = false;
            return true;
        }
        else {
            return false;
        }
    }

    private void setOuttakeState(OuttakeState outtakeState) {
        this.outtakeState = outtakeState;
    }

    private void toSlidePosConstantVel(double targetSlidePos, double targetVel) {
        slideProfile = true;
        slideVel = targetSlidePos < this.targetSlidePos ? -targetVel : targetVel;
        targetMotionProfilePos = targetSlidePos;
    }

    public void stopMotors() {
        verticalLeftMotor.setPower(0);
        verticalRightMotor.setPower(0);
    }

}
