package org.firstinspires.ftc.teamcode.part;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
class IntakeConstants {
    private IntakeConstants() {} // Prevent instantiation

    // Horizontal Linear
    public static final double HOR_LINEAR_INNER_POSE = 0.0;
    public static final double HOR_LINEAR_OUTER_POSE = 0.0;

    public enum HorLinearMode { MANUAL, AUTO, EMERGENCY }
    public static HorLinearMode HOR_LINEAR_MODE = HorLinearMode.AUTO;

    public static final double HOR_LINEAR_AUTO_SPEED = 0.6;
    public static final double HOR_LINEAR_MANUAL_SPEED = 0.3;

    public static final double HOR_LINEAR_kP = 1.0 * 0.001;

    // Eater
    public static final double EATER_ARM_DOWN_POSE = 0.0;
    public static final double EATER_ARM_UP_POSE = 1.0;

    public static final double EATER_SPEED = 0.5;

    public static final double EATER_HAND_DOWN_POSE = 0.5;
    public static final double EATER_HAND_UP_POSE = 1.0;

//    /*
//    mode == MANUAL : hand 각도를 직접 조정
//    mode == AUTO   : hand 각도를 vision 과 연동
//     */
//    public enum EaterMode {MANUAL, AUTO};
//    public static EaterMode EATER_MODE = EaterMode.AUTO;
}

// Main Part
public class Intake implements Part{
    private static final HorizontalLinear horizontalLinear = new HorizontalLinear();
    private static final Eater eater = new Eater();

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        horizontalLinear.init(hardwareMap, telemetry);
        eater.init(hardwareMap, telemetry);
    }

    public void update() {
        horizontalLinear.update();
        eater.update();
    }

    public void stop() {
        horizontalLinear.stop();
        eater.stop();
    }

    public static void cmdAutoStretch() {
        horizontalLinear.cmdSetMode(IntakeConstants.HorLinearMode.AUTO);
        horizontalLinear.cmdStretch();
    }
    public static void cmdAutoRetract() {
        horizontalLinear.cmdSetMode(IntakeConstants.HorLinearMode.AUTO);
        horizontalLinear.cmdRetract();
    }
    public static void cmdManualStretch() {
        horizontalLinear.cmdSetMode(IntakeConstants.HorLinearMode.MANUAL);
        horizontalLinear.cmdStretch();
    }
    public static void cmdManualRetract() {
        horizontalLinear.cmdSetMode(IntakeConstants.HorLinearMode.MANUAL);
        horizontalLinear.cmdRetract();
    }
    public static void cmdManualStop() {
        horizontalLinear.cmdManualStop();
    }

    public static void cmdEaterRun() {
        eater.cmdEaterRun();
    }

    public static void cmdEaterStop() {
        eater.cmdEaterStop();
    }

    public static void cmdHandRotate(double theta) {
        eater.cmdHandRotate(theta);
    }

    public static void cmdArmUp() {
        eater.cmdArmUp();
    }

    public static void cmdArmDown() {
        eater.cmdArmDown();
    }
}

// Sub Part
class HorizontalLinear implements Part {

    private DcMotor motor;
    private double targetPosition = IntakeConstants.HOR_LINEAR_INNER_POSE;
    private boolean isUsingPID = false;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        motor = hardwareMap.get(DcMotor.class, "horizontalLinear");

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        if (this.isUsingPID) {
            double err = targetPosition - motor.getCurrentPosition();
            double power = IntakeConstants.HOR_LINEAR_kP * err;
            if (IntakeConstants.HOR_LINEAR_MODE == IntakeConstants.HorLinearMode.AUTO) {
                if (power > 0) power = Math.min(power, IntakeConstants.HOR_LINEAR_AUTO_SPEED);
                else power = Math.max(power, -IntakeConstants.HOR_LINEAR_AUTO_SPEED);
            } else {
                if (power > 0) power = Math.min(power, IntakeConstants.HOR_LINEAR_MANUAL_SPEED);
                else power = Math.max(power, -IntakeConstants.HOR_LINEAR_MANUAL_SPEED);
            }
            motor.setPower(power);
        }
    }

    public void stop() {
        IntakeConstants.HOR_LINEAR_MODE = IntakeConstants.HorLinearMode.EMERGENCY;
        this.targetPosition = motor.getCurrentPosition();
        this.isUsingPID = false;
        motor.setPower(0);
    }

    public void cmdSetMode(IntakeConstants.HorLinearMode mode) {
        IntakeConstants.HOR_LINEAR_MODE = mode;
        motor.setPower(0);
        if (mode == IntakeConstants.HorLinearMode.AUTO) {
            this.targetPosition = IntakeConstants.HOR_LINEAR_INNER_POSE;
        } else {
            this.targetPosition = motor.getCurrentPosition();
        }
        this.isUsingPID = true;
    }
    public void cmdStretch() {
        if (IntakeConstants.HOR_LINEAR_MODE == IntakeConstants.HorLinearMode.AUTO) {
            this.targetPosition = IntakeConstants.HOR_LINEAR_OUTER_POSE;
            this.isUsingPID = true;
        } else {
            this.isUsingPID = false;
            motor.setPower(IntakeConstants.HOR_LINEAR_MANUAL_SPEED);
        }
    }
    public void cmdRetract() {
        if (IntakeConstants.HOR_LINEAR_MODE == IntakeConstants.HorLinearMode.AUTO) {
            this.targetPosition = IntakeConstants.HOR_LINEAR_INNER_POSE;
            this.isUsingPID = true;
        } else {
            this.isUsingPID = false;
            motor.setPower(-IntakeConstants.HOR_LINEAR_MANUAL_SPEED);
        }
    }
    public void cmdManualStop() {
        if (IntakeConstants.HOR_LINEAR_MODE == IntakeConstants.HorLinearMode.MANUAL) {
            this.isUsingPID = true;
            this.targetPosition = motor.getCurrentPosition();
            motor.setPower(0);
        } else if (IntakeConstants.HOR_LINEAR_MODE == IntakeConstants.HorLinearMode.EMERGENCY) {
            this.isUsingPID = false;
            this.targetPosition = motor.getCurrentPosition();
            motor.setPower(0);
        }
    }
}

// Sub Part
class Eater implements Part {

    private Servo armServo;
    private Servo handServo;
    private CRServo eaterServo;


    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        armServo = hardwareMap.get(Servo.class, "intakeArm");
        handServo = hardwareMap.get(Servo.class, "intakeHand");
        eaterServo = hardwareMap.get(CRServo.class, "intakeEater");
    }

    public void update() {
    }

    public void stop() {
        cmdEaterStop();
        cmdArmDown();
        cmdHandRotate(0.5);
    }

    public void cmdEaterRun(){
        eaterServo.setPower(IntakeConstants.EATER_SPEED);
    }

    public void cmdEaterStop(){
        eaterServo.setPower(0);
    }

    public void cmdHandRotate(double theta){
        handServo.setPosition(theta);
    }

    public void cmdArmUp(){
        handServo.setPosition(IntakeConstants.EATER_HAND_UP_POSE);
        armServo.setPosition(IntakeConstants.EATER_ARM_UP_POSE);
    }

    public void cmdArmDown(){
        handServo.setPosition(IntakeConstants.EATER_HAND_DOWN_POSE);
        armServo.setPosition(IntakeConstants.EATER_ARM_DOWN_POSE);
    }
}