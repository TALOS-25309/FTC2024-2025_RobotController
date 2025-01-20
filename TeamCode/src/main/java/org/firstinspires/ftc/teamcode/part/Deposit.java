package org.firstinspires.ftc.teamcode.part;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.feature.Schedule;
import org.firstinspires.ftc.teamcode.feature.SmartServo;
import org.firstinspires.ftc.teamcode.global.Global;


@Config
class DepositConstants{

    // Vertical Linear
    public static int VER_LINEAR_BOTTOM_POSE = 100;
    public static int VER_LINEAR_HIGH_CHAMBER_POSE = 14500;
    public static int VER_LINEAR_LOW_CHAMBER_POSE = 100;
    public static int VER_LINEAR_HIGH_BASKET_POSE = 50000;
    public static int VER_LINEAR_LOW_BASKET_POSE = 20500;

    public static int VER_LINEAR_TEST_POSE = 20000;

    public enum VerLinearMode { MANUAL, AUTO, EMERGENCY }
    public static DepositConstants.VerLinearMode VER_LINEAR_MODE = DepositConstants.VerLinearMode.AUTO;

    public static double VER_LINEAR_kP = 0.02;
    public static double VER_LINEAR_kI = 0.01;
    public static double VER_LINEAR_kD = 0;
    public static int VER_ERROR_IGNORE = 97;

    public static double VER_LINEAR_AUTO_SPEED = 0.4;
    public static double VER_LINEAR_MANUAL_SPEED = 0.4;

    public static double VER_LIMIT_HIGHTEST = 50000.0;
    public static double VER_LIMIT_LOWEST = 0;

    // Claw
    public static double CLAW_CLAW_OPEN_POS = 0.9;
    public static double CLAW_CLAW_CLOSED_POS = 0.68;

    public static double CLAW_ARM_UP_POS = 0.9;
    public static double CLAW_ARM_DOWN_POS = 0.04;

    public static double CLAW_HAND_UP_POS = 0.65;
    public static double CLAW_HAND_DOWN_POS = 0.65;


    // Deposit Delays
    public static double DELAY_DEPOSIT_SAMPLE_GOTO_LOW = 0.1;
    public static double DELAY_DEPOSIT_SAMPLE_GOTO_HIGH = 0.1;

    public static double DELAY_DEPOSIT_SPECIMEN_GOTO_LOW = 0.1;
    public static double DELAY_DEPOSIT_SPECIMEN_GOTO_HIGH = 0.1;

    public static double DELAY_DEPOSIT_OPEN_CLAW = 0.5;
    public static double DELAY_DEPOSIT_RETRACT_LINEAR_SAMPLE = 3;
    public static double DELAY_DEPOSIT_RETRACT_LINEAR_SPECIMEN_1 = 0.4; //
    public static double DELAY_DEPOSIT_RETRACT_LINEAR_SPECIMEN_2 = 0.1; //

}

public class Deposit implements Part{
    private Telemetry telemetry;

    private static final VerticalLinear verticalLinear = new VerticalLinear();
    private static final Claw claw = new Claw();

    public enum Location {LOW, HIGH}

    private boolean isBusy = false;

    private boolean isStretched = false;
    private boolean isSample = false;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        verticalLinear.init(hardwareMap,telemetry);
        claw.init(hardwareMap,telemetry);
    }

    public void update(){
        if(!Global.IS_TEST)
            verticalLinear.update();
        claw.update();

        telemetry.addData("isStretched", isStretched);
    }

    public void stop() {
        verticalLinear.stop();
        claw.stop();
    }

    public void cmdPIDTest(){
       Schedule.addTask(()->{verticalLinear.cmdStretchForTest();}, 5.0);
    }

    public void cmdDepositSample(Location location){
        if(isBusy) {
            Global.PLAYER2_WARNING = true;
            return;
        }

        isBusy = true;
        isStretched = true;
        isSample = true;

        double delay = 0;

        // 1. Go to Basket
        Schedule.addTask(claw::cmdUp, delay);
        if(location == Location.LOW){
            Schedule.addTask(verticalLinear::cmdStretchToLowBasket, delay);
            delay += DepositConstants.DELAY_DEPOSIT_SAMPLE_GOTO_LOW;
        } else {
            Schedule.addTask(verticalLinear::cmdStretchToHighBasket, delay);
            delay += DepositConstants.DELAY_DEPOSIT_SAMPLE_GOTO_HIGH;
        }

        // 2. End
        Schedule.addTask(() -> {
            isBusy = false;
        }, delay);
    }
    public void cmdDepositSpecimen(Location location){
        if(isBusy) {
            Global.PLAYER2_WARNING = true;
            return;
        }

        isBusy = true;
        isStretched = true;
        isSample = false;

        double delay = 0;

        // 1. Go to Chamber
        Schedule.addTask(claw::cmdUp, delay);
        if (location == Location.LOW) {
            Schedule.addTask(verticalLinear::cmdStretchToLowChamber, delay);
            delay += DepositConstants.DELAY_DEPOSIT_SPECIMEN_GOTO_LOW;
        } else {
            Schedule.addTask(verticalLinear::cmdStretchToHighChamber, delay);
            delay += DepositConstants.DELAY_DEPOSIT_SPECIMEN_GOTO_HIGH;
        }

        // 2. End
        Schedule.addTask(() -> {
            isBusy = false;
        }, delay);
    }
    public void cmdGrabSample() { // related to Intake.cmdTransfer
        if(isBusy) {
            Global.PLAYER2_WARNING = true;
            return;
        }

        isBusy = true;
        isStretched = false;
        Global.ROBOT_STATE = Global.RobotState.DEPOSIT;

        // 0. Wait Until Sample Comes
        double delay = IntakeConstants.DELAY_ARM_COMPLETE;

        // 1. Close Claw
        Schedule.addTask(claw::cmdClose, delay);
        delay = IntakeConstants.DELAY_ARM_REST;

        // 2. End
        Schedule.addTask(() -> {
            isBusy = false;
        }, delay);
    }
    public void cmdReturnSample(){
        if(isBusy) {
            Global.PLAYER2_WARNING = true;
            return;
        }

        isBusy = true;
        isStretched = false;
        Global.ROBOT_STATE = Global.RobotState.NONE;

        double delay = 0;

        // 1. Open Claw
        Schedule.addTask(claw::cmdOpen, delay);
        delay += DepositConstants.DELAY_DEPOSIT_OPEN_CLAW;

        // 2. Retract Linear
        Schedule.addTask(claw::cmdClose,delay);
        Schedule.addTask(claw::cmdDown, delay);
        Schedule.addTask(verticalLinear::cmdRetractToBottom, delay);
        delay += DepositConstants.DELAY_DEPOSIT_RETRACT_LINEAR_SAMPLE;

        // 3. End
        Schedule.addTask(() -> {
            claw.cmdOpen();
            isBusy = false;
        }, delay);
    }
    public void cmdReturnSpecimen(){
        if(isBusy) {
            Global.PLAYER2_WARNING = true;
            return;
        }

        isBusy = true;
        isStretched = false;
        Global.ROBOT_STATE = Global.RobotState.NONE;

        double delay = 0;

        // 1. Retract Linear
        Schedule.addTask(verticalLinear::cmdRetractToBottom, delay);
        delay += DepositConstants.DELAY_DEPOSIT_RETRACT_LINEAR_SPECIMEN_2;

        // 2. Open Claw (during 1)
        Schedule.addTask(claw::cmdOpen, delay);
        Schedule.addTask(claw::cmdDown, delay);
        delay += DepositConstants.DELAY_DEPOSIT_RETRACT_LINEAR_SPECIMEN_1 - DepositConstants.DELAY_DEPOSIT_RETRACT_LINEAR_SPECIMEN_2;

        // 3. End
        Schedule.addTask(() -> {
            isBusy = false;
        }, delay);
    }

    public void cmdManualStretch(){
        verticalLinear.cmdStretch();
    }
    public void cmdManualRetract(){
        verticalLinear.cmdRetract();
    }
    public void cmdManualStop(){
        verticalLinear.cmdManualStop();
    }

    public boolean isStretched() {
        return this.isStretched;
    }
    public boolean isSample() {
        return this.isSample;
    }
}

class VerticalLinear implements Part{

    private Telemetry telemetry;

    private DcMotor motor1, motor2;
    private int targetPosition = DepositConstants.VER_LINEAR_BOTTOM_POSE;

    private boolean isUsingPID = false;

    private double errorSum = 0.0;
    private double previousError = 0.0;
    private double previousTime = 0.0;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        motor1 = hardwareMap.get(DcMotor.class, "verticalLinear1");
        motor2 = hardwareMap.get(DcMotor.class, "verticalLinear2");

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void update() {
        if (this.isUsingPID) {
            double currentTime = (double)System.nanoTime() / 1e9;
            double elapsedTime = currentTime - previousTime;

            int err = targetPosition - getEncoderValue();
            err /= DepositConstants.VER_ERROR_IGNORE;

            double deltaErr = err - this.previousError;
            this.errorSum += err * elapsedTime;
            double differential = deltaErr / elapsedTime;

            previousTime = currentTime;
            previousError = err;

            if(Math.abs(DepositConstants.VER_LINEAR_kP * err) > DepositConstants.VER_LINEAR_AUTO_SPEED) {
                this.errorSum = 0;
                differential = 0;
            }

            double power = DepositConstants.VER_LINEAR_kP * err
                    + DepositConstants.VER_LINEAR_kI * errorSum
                    + DepositConstants.VER_LINEAR_kD * differential;
            if (DepositConstants.VER_LINEAR_MODE == DepositConstants.VerLinearMode.AUTO) {
                if (power > 0) power = Math.min(power, DepositConstants.VER_LINEAR_AUTO_SPEED);
                else power = Math.max(power, -DepositConstants.VER_LINEAR_AUTO_SPEED);
            } else {
                if (power > 0) power = Math.min(power, DepositConstants.VER_LINEAR_MANUAL_SPEED);
                else power = Math.max(power, -DepositConstants.VER_LINEAR_MANUAL_SPEED);
            }
            motor1.setPower(power);
            motor2.setPower(power);

            this.telemetry.addData("Target", targetPosition);
            this.telemetry.addData("Current Pos", getEncoderValue());
            this.telemetry.addData("Power", power);

            telemetry.addData("Integral", this.errorSum);
            telemetry.addData("Differential", differential);
        }
    }

    public void stop() {
        this.targetPosition = getEncoderValue();
        motor1.setPower(0);
        motor2.setPower(0);
        this.isUsingPID = false;
        DepositConstants.VER_LINEAR_MODE = DepositConstants.VerLinearMode.EMERGENCY;
    }

    public void cmdStretchForTest() {
        this.isUsingPID = true;
        DepositConstants.VER_LINEAR_MODE = DepositConstants.VerLinearMode.AUTO;
        this.targetPosition = DepositConstants.VER_LINEAR_HIGH_BASKET_POSE;
        this.initPID();
    }

    public void cmdStretchToHighBasket() {
        if(Global.IS_TEST) return;
        this.isUsingPID = true;
        DepositConstants.VER_LINEAR_MODE = DepositConstants.VerLinearMode.AUTO;
        this.targetPosition = DepositConstants.VER_LINEAR_HIGH_BASKET_POSE;
        this.initPID();
    }

    public void cmdStretchToLowBasket() {
        if(Global.IS_TEST) return;
        this.isUsingPID = true;
        DepositConstants.VER_LINEAR_MODE = DepositConstants.VerLinearMode.AUTO;
        this.targetPosition = DepositConstants.VER_LINEAR_LOW_BASKET_POSE;
        this.initPID();
    }

    public void cmdStretchToLowChamber() {
        if(Global.IS_TEST) return;
        this.isUsingPID = true;
        DepositConstants.VER_LINEAR_MODE = DepositConstants.VerLinearMode.AUTO;
        this.targetPosition = DepositConstants.VER_LINEAR_LOW_CHAMBER_POSE;
        this.initPID();
    }

    public void cmdStretchToHighChamber() {
        if(Global.IS_TEST) return;
        this.isUsingPID = true;
        DepositConstants.VER_LINEAR_MODE = DepositConstants.VerLinearMode.AUTO;
        this.targetPosition = DepositConstants.VER_LINEAR_HIGH_CHAMBER_POSE;
        this.initPID();
    }

    public void cmdRetractToBottom() {
        if(Global.IS_TEST) return;
        this.isUsingPID = true;
        DepositConstants.VER_LINEAR_MODE = DepositConstants.VerLinearMode.AUTO;
        this.targetPosition = DepositConstants.VER_LINEAR_BOTTOM_POSE;
        this.initPID();
    }

    public void cmdStretch() {
        if(Global.IS_TEST) return;
        this.isUsingPID = false;
        DepositConstants.VER_LINEAR_MODE = DepositConstants.VerLinearMode.MANUAL;
        motor1.setPower(DepositConstants.VER_LINEAR_MANUAL_SPEED);
        motor2.setPower(DepositConstants.VER_LINEAR_MANUAL_SPEED);

        if (getEncoderValue() > DepositConstants.VER_LIMIT_HIGHTEST) {
            motor1.setPower(0);
            motor2.setPower(0);
        }
    }

    public void cmdRetract() {
        if(Global.IS_TEST) return;
        this.isUsingPID = false;
        DepositConstants.VER_LINEAR_MODE = DepositConstants.VerLinearMode.MANUAL;
        motor1.setPower(-DepositConstants.VER_LINEAR_MANUAL_SPEED);
        motor2.setPower(-DepositConstants.VER_LINEAR_MANUAL_SPEED);

        if (getEncoderValue() < DepositConstants.VER_LIMIT_LOWEST) {
            motor1.setPower(0);
            motor2.setPower(0);
        }
    }

    public void cmdManualStop() {
        if (DepositConstants.VER_LINEAR_MODE == DepositConstants.VerLinearMode.MANUAL) {
            if (!this.isUsingPID) {
                this.targetPosition = getEncoderValue();
                this.initPID();
                motor1.setPower(0);
                motor2.setPower(0);
            }
            this.isUsingPID = true;
        } else if (DepositConstants.VER_LINEAR_MODE == DepositConstants.VerLinearMode.EMERGENCY) {
            if (!this.isUsingPID) {
                this.targetPosition = getEncoderValue();
                this.initPID();
            }
            this.isUsingPID = false;
            motor1.setPower(0);
            motor2.setPower(0);
        }
    }

    private int getEncoderValue() {
        return motor1.getCurrentPosition();
    }

    private void initPID() {
        this.errorSum = 0.0;
        this.previousError = 0.0;
        this.previousTime = (double)System.nanoTime() / 1e9;
    }
}

class Claw implements Part{

    private Telemetry telemetry;

    private SmartServo servoClaw;
    private SmartServo servoHand;
    private SmartServo servoArm1, servoArm2;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        servoHand = new SmartServo(hardwareMap.get(Servo.class, "depositHand"), "depositHand");
        servoArm1 = new SmartServo(hardwareMap.get(Servo.class, "depositArm1"), "depositArm1");
        servoArm2 = new SmartServo(hardwareMap.get(Servo.class, "depositArm2"), "depositArm2");
        servoClaw = new SmartServo(hardwareMap.get(Servo.class, "depositClaw"), "depositClaw");

        servoClaw.servo().setDirection(Servo.Direction.FORWARD);
        servoHand.servo().setDirection(Servo.Direction.FORWARD);
        servoArm1.servo().setDirection(Servo.Direction.FORWARD);
        servoArm2.servo().setDirection(Servo.Direction.REVERSE);

        servoClaw.setPosition(DepositConstants.CLAW_CLAW_OPEN_POS);
        servoHand.setPosition(DepositConstants.CLAW_HAND_DOWN_POS);
        servoArm1.setPosition(DepositConstants.CLAW_ARM_DOWN_POS);
        servoArm2.setPosition(DepositConstants.CLAW_ARM_DOWN_POS);
    }

    public void update() {

    }

    public void stop() {
        cmdOpen();
    }

    public void cmdOpen() {
        servoClaw.setPosition(DepositConstants.CLAW_CLAW_OPEN_POS);
    }

    public void cmdClose() {
        servoClaw.setPosition(DepositConstants.CLAW_CLAW_CLOSED_POS);
    }

    public void cmdUp() {
        servoHand.setPosition(DepositConstants.CLAW_HAND_UP_POS);
        servoArm1.setPosition(DepositConstants.CLAW_ARM_UP_POS);
        servoArm2.setPosition(DepositConstants.CLAW_ARM_UP_POS);
    }

    public void cmdDown() {
        servoHand.setPosition(DepositConstants.CLAW_HAND_DOWN_POS);
        servoArm1.setPosition(DepositConstants.CLAW_ARM_DOWN_POS);
        servoArm2.setPosition(DepositConstants.CLAW_ARM_DOWN_POS);
    }
}


