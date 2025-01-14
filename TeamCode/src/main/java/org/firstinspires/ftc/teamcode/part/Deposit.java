package org.firstinspires.ftc.teamcode.part;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.feature.Schedule;


@Config
class DepositConstants{
    // Vertical Linear
    public static final Double VER_LINEAR_BOTTOM_POSE = 0.0;
    public static final Double VER_LINEAR_HIGH_CHAMBER_POSE = 1.0;
    public static final Double VER_LINEAR_LOW_CHAMBER_POSE = 1.0;
    public static final Double VER_LINEAR_HIGH_BASKET_POSE = 0.0;
    public static final Double VER_LINEAR_LOW_BASKET_POSE = 0.0;

    public enum VerLinearMode { MANUAL, AUTO, EMERGENCY }
    public static DepositConstants.VerLinearMode VER_LINEAR_MODE = DepositConstants.VerLinearMode.AUTO;

    public static final Double VER_LINEAR_kP = 1.0 * 0.001;

    public static final double HOR_LINEAR_AUTO_SPEED = 0.6;
    public static final double HOR_LINEAR_MANUAL_SPEED = 0.3;

    // Claw
    public static final Double CLAW_OPEN_POS = 1.0;
    public static final Double CLAW_CLOSED_POS = 0.0;

    public static final Double CLAW_UP_POS = 1.0;
    public static final Double CLAW_DOWN_POS = 0.0;


    // Deposit Delays
    public static final long DEPOSIT_DELAY_CLOSE_CLAW = 100;

    public static final long DEPOSIT_SAMPLE_DELAY_GOTO_LOW = 100;
    public static final long DEPOSIT_SAMPLE_DELAY_GOTO_HIGH = 100;

    public static final long DEPOSIT_SPECIMEN_DELAY_GOTO_LOW = 100;
    public static final long DEPOSIT_SPECIMEN_DELAY_GOTO_HIGH = 100;

    public static final long DEPOSIT_DELAY_OPEN_CLAW = 100;
    public static final long DEPOSIT_DELAY_RETRACT_LINEAR = 100;

}

public class Deposit implements Part{
    public static final VerticalLinear verticalLinear = new VerticalLinear();
    public static final Claw claw = new Claw();

    public enum Location {LOW, HIGH}

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        verticalLinear.init(hardwareMap,telemetry);
        claw.init(hardwareMap,telemetry);
    }

    public void update(){
        verticalLinear.update();
        claw.update();
    }

    public void stop() {
        verticalLinear.stop();
        claw.stop();
    }

    public void cmdDepositSample(Location location){
        if(verticalLinear.isBusy() || claw.isBusy()) return;

        verticalLinear.setBusy(true);
        claw.setBusy(true);

        double delay = 0;

//        // 1. Close Claw
//        Schedule.addTask(claw::cmdClose, delay);
//        delay += DepositConstants.DEPOSIT_DELAY_CLOSE_CLAW;

        // 2. Go to Basket
        Schedule.addTask(claw::cmdUp, delay);
        if(location == Location.LOW){
            Schedule.addTask(verticalLinear::cmdStretchToLowBasket, delay);
            delay += DepositConstants.DEPOSIT_SAMPLE_DELAY_GOTO_LOW;
        } else {
            Schedule.addTask(verticalLinear::cmdStretchToHighBasket, delay);
            delay += DepositConstants.DEPOSIT_SAMPLE_DELAY_GOTO_HIGH;
        }

        // 3. End
        Schedule.addTask(() -> {
            verticalLinear.setBusy(false);
            claw.setBusy(false);
        }, delay);
    }
    public void cmdDepositSpecimen(Location location){
        if(verticalLinear.isBusy() || claw.isBusy()) return;

        verticalLinear.setBusy(true);
        claw.setBusy(true);

        double delay = 0;

//        // 1. Close Claw
//        Schedule.addTask(claw::cmdClose, delay);
//        delay += DepositConstants.DEPOSIT_DELAY_CLOSE_CLAW;

        // 2. Go to Chamber
        Schedule.addTask(claw::cmdUp, delay);
        if (location == Location.LOW) {
            Schedule.addTask(verticalLinear::cmdStretchToLowChamber, delay);
            delay += DepositConstants.DEPOSIT_SPECIMEN_DELAY_GOTO_LOW;
        } else {
            Schedule.addTask(verticalLinear::cmdStretchToHighChamber, delay);
            delay += DepositConstants.DEPOSIT_SPECIMEN_DELAY_GOTO_HIGH;
        }

        // 3. End
        Schedule.addTask(() -> {
            verticalLinear.setBusy(false);
            claw.setBusy(false);
        }, delay);
    }
    public void cmdGrabSample() { // related to Intake.cmdTransfer
        if (verticalLinear.isBusy() || claw.isBusy()) return;

        claw.setBusy(true);

        // 0. Wait Until Sample Comes
        double delay = IntakeConstants.TIME4_1 + IntakeConstants.TIME4_2;

        // 1. Close Claw
        Schedule.addTask(claw::cmdClose, delay);
        delay += DepositConstants.DEPOSIT_DELAY_CLOSE_CLAW;

        // 2. End
        Schedule.addTask(() -> {
            verticalLinear.setBusy(false);
            claw.setBusy(false);
        }, delay);
    }
    public void cmdReturn(){
        if(verticalLinear.isBusy() || claw.isBusy()) return;

        verticalLinear.setBusy(true);
        claw.setBusy(true);

        double delay = 0;

        // 1. Open Claw
        Schedule.addTask(claw::cmdOpen, delay);
        delay += DepositConstants.DEPOSIT_DELAY_OPEN_CLAW;

        // 2. Retract Linear
        Schedule.addTask(claw::cmdDown, delay);
        Schedule.addTask(verticalLinear::cmdRetractToBottom, delay);

        // 3. End
        Schedule.addTask(() -> {
            verticalLinear.setBusy(false);
            claw.setBusy(false);
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
}

class VerticalLinear implements Part{

    private DcMotor motor;
    private double targetPosition = DepositConstants.VER_LINEAR_BOTTOM_POSE;

    private boolean isBusy = false;
    private boolean isUsingPID = false;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        motor = hardwareMap.get(DcMotor.class, "verticalLinear");

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        if (this.isUsingPID) {
            double err = targetPosition - motor.getCurrentPosition();
            double power = DepositConstants.VER_LINEAR_kP * err;
            if (DepositConstants.VER_LINEAR_MODE == DepositConstants.VerLinearMode.AUTO) {
                if (power > 0) power = Math.min(power, DepositConstants.HOR_LINEAR_AUTO_SPEED);
                else power = Math.max(power, -DepositConstants.HOR_LINEAR_AUTO_SPEED);
            } else {
                if (power > 0) power = Math.min(power, DepositConstants.HOR_LINEAR_MANUAL_SPEED);
                else power = Math.max(power, -DepositConstants.HOR_LINEAR_MANUAL_SPEED);
            }
            motor.setPower(power);
        }
    }

    public void stop() {
        this.targetPosition = motor.getCurrentPosition();
        motor.setPower(0);
        this.isUsingPID = false;
        DepositConstants.VER_LINEAR_MODE = DepositConstants.VerLinearMode.EMERGENCY;
        setBusy(false);
    }

    public void cmdStretchToHighBasket() {
        this.targetPosition = DepositConstants.VER_LINEAR_HIGH_BASKET_POSE;
    }

    public void cmdStretchToLowBasket() {
        this.targetPosition = DepositConstants.VER_LINEAR_LOW_BASKET_POSE;
    }

    public void cmdStretchToLowChamber() {
        this.targetPosition = DepositConstants.VER_LINEAR_LOW_CHAMBER_POSE;
    }

    public void cmdStretchToHighChamber() {
        this.targetPosition = DepositConstants.VER_LINEAR_HIGH_CHAMBER_POSE;
    }

    public void cmdRetractToBottom() {
        this.targetPosition = DepositConstants.VER_LINEAR_BOTTOM_POSE;
    }

    public void cmdStretch() {
        this.isUsingPID = false;
        motor.setPower(IntakeConstants.HOR_LINEAR_MANUAL_SPEED);
    }

    public void cmdRetract() {
        this.isUsingPID = false;
        motor.setPower(-IntakeConstants.HOR_LINEAR_MANUAL_SPEED);
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

    public void setBusy(boolean busy) {
        this.isBusy = busy;
    }

    public boolean isBusy() {
        return this.isBusy;
    }
}

class Claw implements Part{

    private Servo servoHand;
    private Servo servoArm;

    private boolean isBusy = false;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        servoHand = hardwareMap.get(Servo.class, "servoHandClaw");
        servoArm = hardwareMap.get(Servo.class, "servoArmClaw");
    }

    public void update() {

    }

    public void stop() {
        cmdOpen();
        setBusy(false);
    }

    public void cmdOpen() {
        servoHand.setPosition(DepositConstants.CLAW_OPEN_POS);
    }

    public void cmdClose() {
        servoHand.setPosition(DepositConstants.CLAW_CLOSED_POS);
    }

    public void cmdUp() {
        servoArm.setPosition(DepositConstants.CLAW_UP_POS);
    }

    public void cmdDown() {
        servoArm.setPosition(DepositConstants.CLAW_DOWN_POS);
    }

    public void setBusy(boolean busy) {
        isBusy = busy;
    }

    public boolean isBusy() {
        return this.isBusy;
    }
}


