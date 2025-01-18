package org.firstinspires.ftc.teamcode.part;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
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
    public static double VER_LINEAR_BOTTOM_POSE = 100.0;
    public static double VER_LINEAR_HIGH_CHAMBER_POSE = 20000.0;
    public static double VER_LINEAR_LOW_CHAMBER_POSE = 20000.0;
    public static double VER_LINEAR_HIGH_BASKET_POSE = 20000.0;
    public static double VER_LINEAR_LOW_BASKET_POSE = 20000.0;

    public enum VerLinearMode { MANUAL, AUTO, EMERGENCY }
    public static DepositConstants.VerLinearMode VER_LINEAR_MODE = DepositConstants.VerLinearMode.AUTO;

    public static double VER_LINEAR_kP = 2.0 * 0.001;

    public static double VER_LINEAR_AUTO_SPEED = 0.8;
    public static double VER_LINEAR_MANUAL_SPEED = 0.5;

    public static double VER_HIGHTEST_LIMIT = 50000.0;
    public static double VER_LOWEST_LIMIT = 0;

    // Claw
    public static double CLAW_OPEN_POS = 0.6;
    public static double CLAW_CLOSED_POS = 0.4;

    public static double CLAW_ARM_UP_POS = 0;
    public static double CLAW_ARM_DOWN_POS = 1;

    public static double CLAW_HAND_UP_POS = 0.7;
    public static double CLAW_HAND_DOWN_POS = 0.7;


    // Deposit Delays
    public static double DEPOSIT_SAMPLE_DELAY_GOTO_LOW = 0.1;
    public static double DEPOSIT_SAMPLE_DELAY_GOTO_HIGH = 0.1;

    public static double DEPOSIT_SPECIMEN_DELAY_GOTO_LOW = 0.1;
    public static double DEPOSIT_SPECIMEN_DELAY_GOTO_HIGH = 0.1;

    public static double DEPOSIT_DELAY_OPEN_CLAW = 0.1;
    public static double DEPOSIT_DELAY_RETRACT_LINEAR = 0.1;

}

public class Deposit implements Part{
    private Telemetry telemetry;

    private static final VerticalLinear verticalLinear = new VerticalLinear();
    private static final Claw claw = new Claw();

    public enum Location {LOW, HIGH}

    private boolean isBusy = false;

    private boolean isStretched = false;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        verticalLinear.init(hardwareMap,telemetry);
        claw.init(hardwareMap,telemetry);
    }

    public void update(){
        verticalLinear.update();
        claw.update();

        telemetry.addData("isStretched", isStretched);
    }

    public void stop() {
        verticalLinear.stop();
        claw.stop();
    }

    public void cmdDepositSample(Location location){
        if(isBusy) {
            Global.PLAYER2_WARNING = true;
            return;
        }

        isBusy = true;
        isStretched = true;

        double delay = 0;

        // 1. Go to Basket
        Schedule.addTask(claw::cmdUp, delay);
        if(location == Location.LOW){
            Schedule.addTask(verticalLinear::cmdStretchToLowBasket, delay);
            delay += DepositConstants.DEPOSIT_SAMPLE_DELAY_GOTO_LOW;
        } else {
            Schedule.addTask(verticalLinear::cmdStretchToHighBasket, delay);
            delay += DepositConstants.DEPOSIT_SAMPLE_DELAY_GOTO_HIGH;
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

        double delay = 0;

        // 1. Go to Chamber
        Schedule.addTask(claw::cmdUp, delay);
        if (location == Location.LOW) {
            Schedule.addTask(verticalLinear::cmdStretchToLowChamber, delay);
            delay += DepositConstants.DEPOSIT_SPECIMEN_DELAY_GOTO_LOW;
        } else {
            Schedule.addTask(verticalLinear::cmdStretchToHighChamber, delay);
            delay += DepositConstants.DEPOSIT_SPECIMEN_DELAY_GOTO_HIGH;
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
    public void cmdReturn(){
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
        delay += DepositConstants.DEPOSIT_DELAY_OPEN_CLAW;

        // 2. Retract Linear
        Schedule.addTask(claw::cmdDown, delay);
        Schedule.addTask(verticalLinear::cmdRetractToBottom, delay);

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
}

class VerticalLinear implements Part{

    private Telemetry telemetry;

    private DcMotor motor1, motor2;
    private double targetPosition = DepositConstants.VER_LINEAR_BOTTOM_POSE;

    private boolean isUsingPID = false;

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
            double err = targetPosition - motor1.getCurrentPosition();
            double power = DepositConstants.VER_LINEAR_kP * err;
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
            this.telemetry.addData("Current Pos", motor1.getCurrentPosition());
            this.telemetry.addData("Power", power);
        }
    }

    public void stop() {
        this.targetPosition = motor1.getCurrentPosition();
        motor1.setPower(0);
        motor2.setPower(0);
        this.isUsingPID = false;
        DepositConstants.VER_LINEAR_MODE = DepositConstants.VerLinearMode.EMERGENCY;
    }

    public void cmdStretchToHighBasket() {
        this.isUsingPID = true;
        DepositConstants.VER_LINEAR_MODE = DepositConstants.VerLinearMode.AUTO;
        this.targetPosition = DepositConstants.VER_LINEAR_HIGH_BASKET_POSE;
    }

    public void cmdStretchToLowBasket() {
        this.isUsingPID = true;
        DepositConstants.VER_LINEAR_MODE = DepositConstants.VerLinearMode.AUTO;
        this.targetPosition = DepositConstants.VER_LINEAR_LOW_BASKET_POSE;
    }

    public void cmdStretchToLowChamber() {
        this.isUsingPID = true;
        DepositConstants.VER_LINEAR_MODE = DepositConstants.VerLinearMode.AUTO;
        this.targetPosition = DepositConstants.VER_LINEAR_LOW_CHAMBER_POSE;
    }

    public void cmdStretchToHighChamber() {
        this.isUsingPID = true;
        DepositConstants.VER_LINEAR_MODE = DepositConstants.VerLinearMode.AUTO;
        this.targetPosition = DepositConstants.VER_LINEAR_HIGH_CHAMBER_POSE;
    }

    public void cmdRetractToBottom() {
        this.isUsingPID = true;
        DepositConstants.VER_LINEAR_MODE = DepositConstants.VerLinearMode.AUTO;
        this.targetPosition = DepositConstants.VER_LINEAR_BOTTOM_POSE;
    }

    public void cmdStretch() {
        this.isUsingPID = false;
        DepositConstants.VER_LINEAR_MODE = DepositConstants.VerLinearMode.MANUAL;
        motor1.setPower(DepositConstants.VER_LINEAR_MANUAL_SPEED);
        motor2.setPower(DepositConstants.VER_LINEAR_MANUAL_SPEED);
    }

    public void cmdRetract() {
        this.isUsingPID = false;
        DepositConstants.VER_LINEAR_MODE = DepositConstants.VerLinearMode.MANUAL;
        motor1.setPower(-DepositConstants.VER_LINEAR_MANUAL_SPEED);
        motor2.setPower(-DepositConstants.VER_LINEAR_MANUAL_SPEED);
    }

    public void cmdManualStop() {
        if (DepositConstants.VER_LINEAR_MODE == DepositConstants.VerLinearMode.MANUAL) {
            this.isUsingPID = true;
            this.targetPosition = motor1.getCurrentPosition();
            motor1.setPower(0);
            motor2.setPower(0);
        } else if (DepositConstants.VER_LINEAR_MODE == DepositConstants.VerLinearMode.EMERGENCY) {
            this.isUsingPID = false;
            this.targetPosition = motor1.getCurrentPosition();
            motor1.setPower(0);
            motor2.setPower(0);
        }
    }
}

class Claw implements Part{

    private Telemetry telemetry;

    private SmartServo servoClaw;
    private SmartServo servoHand;
    private SmartServo servoArm1, servoArm2;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        servoHand = new SmartServo(hardwareMap.get(Servo.class, "depositHand"));
        servoArm1 = new SmartServo(hardwareMap.get(Servo.class, "depositArm1"));
        servoArm2 = new SmartServo(hardwareMap.get(Servo.class, "depositArm2"));
        servoClaw = new SmartServo(hardwareMap.get(Servo.class, "depositClaw"));

        servoClaw.Servo().setDirection(Servo.Direction.FORWARD);
        servoHand.Servo().setDirection(Servo.Direction.FORWARD);
        servoArm1.Servo().setDirection(Servo.Direction.FORWARD);
        servoArm2.Servo().setDirection(Servo.Direction.REVERSE);

        servoClaw.setPosition(DepositConstants.CLAW_OPEN_POS);
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
        servoClaw.setPosition(DepositConstants.CLAW_OPEN_POS);
    }

    public void cmdClose() {
        servoClaw.setPosition(DepositConstants.CLAW_CLOSED_POS);
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


