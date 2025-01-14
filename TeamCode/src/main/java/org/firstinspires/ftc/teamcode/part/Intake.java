package org.firstinspires.ftc.teamcode.part;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.feature.Schedule;
import org.firstinspires.ftc.teamcode.feature.Vision;

@Config
class IntakeConstants {
    private IntakeConstants() {} // Prevent instantiation

    // Intake
    public static final double TIME1_1 = 100;

    public static final double TIME2_1 = 100;
    public static final double TIME2_2 = 100;
    public static final double TIME2_3 = 100;

    public static final double TIME3_1 = 100;
    public static final double TIME3_2 = 100;

    public static final double TIME4_1 = 100;
    public static final double TIME4_2 = 100;
    public static final double TIME4_3 = 100;


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

    public static final double EATER_HAND_SPEED = 0.02;

}

// Main Part
public class Intake implements Part{
    private static final HorizontalLinear horizontalLinear = new HorizontalLinear();
    private static final Eater eater = new Eater();

    private static int currentStep = 0;
    private static int previousStep = 0;
    private int[] nextStep = {1, 3, 1, 0};
    private int[] backStep = {0, 2, 0, 3};

    private Vision.SampleColor targetColor = Vision.SampleColor.YELLOW;

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

    // functions
    public void cmdStretchLinear(){
        if(horizontalLinear.isBusy() || eater.isBusy()) return;

        horizontalLinear.setBusy(true);
        eater.setBusy(true);

        double delay = 0;

        // 1. Stretch Linear
        Schedule.addTask(Intake::cmdAutoStretch, delay);
        delay += IntakeConstants.TIME1_1;

        // 2. End
        Schedule.addTask(()->{
            horizontalLinear.setBusy(false);
            eater.setBusy(false);
        }, delay);
    }
    public void cmdEat(){
        if(horizontalLinear.isBusy() || eater.isBusy()) return;

        horizontalLinear.setBusy(true);
        eater.setBusy(true);

        double delay = 0;

        horizontalLinear.cmdSetMode(IntakeConstants.HorLinearMode.MANUAL);

        // 1. Lower Eater
        Schedule.addTask(eater::cmdArmDown, delay);
        delay += IntakeConstants.TIME2_1;

        // 2. Run Eater
        Schedule.addTask(() -> {eater.cmdEaterRun(true);}, delay);
        delay += IntakeConstants.TIME2_2;

        // 3. End
        Schedule.addTask(() -> {
            horizontalLinear.setBusy(false);
            eater.setBusy(false);
        }, delay);
    }
    public void cmdVomit(){
        if(horizontalLinear.isBusy() || eater.isBusy()) return;

        horizontalLinear.setBusy(true);
        eater.setBusy(true);

        double delay = 0;

        horizontalLinear.cmdSetMode(IntakeConstants.HorLinearMode.MANUAL);

        // 1. Remove from Eater
        Schedule.addTask(() -> {eater.cmdEaterRun(false);}, delay);
        delay += IntakeConstants.TIME3_1;

        // 2. Raise Eater
        Schedule.addTask(eater::cmdArmUp, delay);
        delay += IntakeConstants.TIME3_2;

        // 3. End
        Schedule.addTask(() -> {
            eater.stop();
            horizontalLinear.setBusy(false);
            eater.setBusy(false);
        }, delay);
    }
    public void cmdTransfer(){
        if(horizontalLinear.isBusy() || eater.isBusy()) return;

        horizontalLinear.setBusy(true);
        eater.setBusy(true);

        double delay = 0;

        // 1. Raise Eater
        Schedule.addTask(eater::cmdArmUp, delay);
        delay += IntakeConstants.TIME4_1;

        // 2. Retract Horizontal Linear
        Schedule.addTask(Intake::cmdAutoRetract, delay);
        delay += IntakeConstants.TIME4_2;

        // 3. Close Claw
        // executed on Deposit.cmdGrabSample
        delay += DepositConstants.DEPOSIT_DELAY_CLOSE_CLAW;

        // 4. Remove from Eater
//        Schedule.addTask(()->{eater.cmdEaterRun(false);},delay);
        Schedule.addTask(eater::cmdEaterStop,delay);
        delay += IntakeConstants.TIME4_3;

        // 5. End
        Schedule.addTask(()->{
            horizontalLinear.setBusy(false);
            eater.setBusy(false);
        }, delay);

    }

    public static void cmdAutoStretch() {
        horizontalLinear.cmdSetMode(IntakeConstants.HorLinearMode.AUTO);
        horizontalLinear.cmdStretch();
    }
    public static void cmdAutoRetract() {
        horizontalLinear.cmdSetMode(IntakeConstants.HorLinearMode.AUTO);
        horizontalLinear.cmdRetract();
    }
    public static void cmdAutoRotate() {
        eater.cmdHandAutoRotate();
    }
    public void cmdManualStretch() {
        horizontalLinear.cmdSetMode(IntakeConstants.HorLinearMode.MANUAL);
        horizontalLinear.cmdStretch();
    }
    public void cmdManualRetract() {
        horizontalLinear.cmdSetMode(IntakeConstants.HorLinearMode.MANUAL);
        horizontalLinear.cmdRetract();
    }
    public void cmdManualStop() {
        horizontalLinear.cmdManualStop();
    }
    public void cmdManualRotate(int direction) {
        eater.cmdHandManualRotate(direction);
    }

    public void setTargetColor(Vision.SampleColor color){
        targetColor = color;
    }

    public void cmdRunNextStep(){
        currentStep = nextStep[currentStep];
        cmdRunCurrentStep();
    }
    public void cmdRunPrevStep(){
        currentStep = backStep[currentStep];
        cmdRunCurrentStep();
    }
    public void cmdRunCurrentStep(){
        if (currentStep == previousStep) return;
        switch (currentStep){
            case 0: cmdStretchLinear();
            case 1: cmdEat();
            case 2: cmdVomit();
            case 3: cmdTransfer();
        }
    }
    public int cmdGetCurrentStep(){
        return currentStep;
    }
}

// Sub Part
class HorizontalLinear implements Part {

    private DcMotor motor;
    private double targetPosition = IntakeConstants.HOR_LINEAR_INNER_POSE;
    private boolean isUsingPID = false;
    private boolean isBusy = false;

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
    public void setBusy(boolean busy){
        isBusy = busy;
    }
    public boolean isBusy(){
        return isBusy;
    }
}

// Sub Part
class Eater implements Part {

    private Servo armServo;
    private Servo handServo;
    private CRServo eaterServo;
    private boolean isBusy;

    private double targetAngle;


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
        cmdHandManualRotate(0);
    }

    public void cmdEaterRun(boolean inside){
        eaterServo.setPower(inside ? IntakeConstants.EATER_SPEED : -IntakeConstants.EATER_SPEED);
    }
    public void cmdEaterStop(){
        eaterServo.setPower(0);
    }
    public void cmdHandAutoRotate(){
        Vision.Sample target = Vision.detectTarget();
        if (target != null){
            targetAngle = target.angle;
        }
        handServo.setPosition(targetAngle);
    }
    public void cmdHandManualRotate(int direction){
        targetAngle += direction * IntakeConstants.EATER_HAND_SPEED;

        if (targetAngle < 0) targetAngle = 0;
        else if (targetAngle > 1) targetAngle = 1;

        handServo.setPosition(targetAngle);
    }
    public void cmdArmUp(){
        armServo.setPosition(IntakeConstants.EATER_ARM_UP_POSE);
    }
    public void cmdArmDown(){
        armServo.setPosition(IntakeConstants.EATER_ARM_DOWN_POSE);
    }
    public void setBusy(boolean busy){
        isBusy = busy;
    }
    public boolean isBusy(){
        return isBusy;
    }
}