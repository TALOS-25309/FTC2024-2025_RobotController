package org.firstinspires.ftc.teamcode.part;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.Executors;
import java.util.concurrent.ExecutorService;



@Config
class DepositConstants{
    // vetical linear
    public static final Double VER_LINEAR_BOTTOM_POS = 0.0;
    public static final Double VER_LINEAR_CHAMBER_POS = 1.0;
    public static final Double VER_LINEAR_HIGH_BASKET_POS = 0.0;
    public static final Double VER_LINEAR_LOW_BASKET_POS = 0.0;

    public static final Double VER_LINEAR_kP = 0.05;

    // claw
    public static final Double CLAW_OPEN_POS = 1.0;
    public static final Double CLAW_CLOSED_POS = 0.0;

    public static final Double CLAW_UP_POS = 1.0;
    public static final Double CLAW_DOWN_POS = 0.0;


    // sample
    public static final long SAMPLE_TIME_0 = 100;
    public static final long SAMPLE_TIME_1 = 100;
    public static final long SAMPLE_TIME_2 = 100;
    public static final long SAMPLE_TIME_3 = 100;

    // specimen
    public static final long SPECIMEN_TIME_0 = 100;
    public static final long SPECIMEN_TIME_1 = 100;
    public static final long SPECIMEN_TIME_2 = 100;
    public static final long SPECIMEN_TIME_3 = 100;
    public static final long SPECIMEN_TIME_4 = 100;

}

public class DepositArm implements Part{

    public static final VerticalLinear verticalLinear = new VerticalLinear();
    public static final Claw claw = new Claw();

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

    public static void cmdSample(){
        if(verticalLinear.isBusy || claw.isBusy) return;

        ExecutorService executorService = Executors.newCachedThreadPool();

        executorService.submit(() -> {
            verticalLinear.setBusy(true);
            claw.setBusy(true);

            try{
                claw.cmdClose();
                Thread.sleep(DepositConstants.SAMPLE_TIME_0);

                verticalLinear.cmdStretchToHighBasket();
                claw.cmdUp();
                Thread.sleep(DepositConstants.SAMPLE_TIME_1);

                claw.cmdOpen();
                Thread.sleep(DepositConstants.SAMPLE_TIME_2);

                verticalLinear.cmdRetractToBottom();
                claw.cmdDown();
                Thread.sleep(DepositConstants.SAMPLE_TIME_3);
            }
            catch (InterruptedException e){
                e.printStackTrace();
            }

            verticalLinear.setBusy(false);
            claw.setBusy(false);
        });
    }

    public void cmdSpecimen(){
        if(verticalLinear.isBusy || claw.isBusy) return;

        ExecutorService executorService = Executors.newCachedThreadPool();

        executorService.submit(() -> {
            verticalLinear.setBusy(true);
            claw.setBusy(true);

            try {
                claw.cmdClose();
                Thread.sleep(DepositConstants.SPECIMEN_TIME_0);

                verticalLinear.cmdStretchToChamber();
                claw.cmdUp();
                Thread.sleep(DepositConstants.SPECIMEN_TIME_1);

                verticalLinear.cmdRetractToBottom();
                Thread.sleep(DepositConstants.SPECIMEN_TIME_2);

                claw.cmdOpen();
                Thread.sleep(DepositConstants.SPECIMEN_TIME_3);

                claw.cmdDown();
                Thread.sleep(DepositConstants.SPECIMEN_TIME_4);
            }
            catch (Exception e){
                e.printStackTrace();
            }

            verticalLinear.setBusy(false);
            claw.setBusy(false);
        });

    }
}

class VerticalLinear implements Part{

    private DcMotor motor;
    private double targetPos = DepositConstants.VER_LINEAR_BOTTOM_POS;

    private enum LINEARPOS {STRETCHED, RETRACTED, MOVING};
    private LINEARPOS LinearPos = LINEARPOS.STRETCHED;

    public boolean isBusy = false;


    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        motor = hardwareMap.get(DcMotor.class, "verticalLinear");

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        double err = targetPos - motor.getCurrentPosition();
        double power = DepositConstants.VER_LINEAR_kP * err;
        motor.setPower(power);
    }

    public void stop(){
        this.targetPos = motor.getCurrentPosition();
        motor.setPower(0);
        setBusy(false);
    }

    public void cmdStretchToHighBasket(){
        this.targetPos = DepositConstants.VER_LINEAR_HIGH_BASKET_POS;
        LinearPos = LINEARPOS.STRETCHED;
    }

    public void cmdStretchToLowBasket(){
        this.targetPos = DepositConstants.VER_LINEAR_LOW_BASKET_POS;
        LinearPos = LINEARPOS.STRETCHED;
    }

    public void cmdStretchToChamber(){
        this.targetPos = DepositConstants.VER_LINEAR_CHAMBER_POS;
        LinearPos = LINEARPOS.STRETCHED;
    }

    public void cmdRetractToBottom(){
        this.targetPos = DepositConstants.VER_LINEAR_BOTTOM_POS;
        LinearPos = LINEARPOS.RETRACTED;
    }

    public void setBusy(boolean busy){
        this.isBusy = busy;
    }

}

class Claw implements Part{

    private Servo servoHand;
    private Servo servoArm;

    public boolean isBusy = false;

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

    public void cmdOpen(){
        servoHand.setPosition(DepositConstants.CLAW_OPEN_POS);
    }

    public void cmdClose(){
        servoHand.setPosition(DepositConstants.CLAW_CLOSED_POS);
    }

    public void cmdUp(){
        servoArm.setPosition(DepositConstants.CLAW_UP_POS);
    }

    public void cmdDown(){
        servoArm.setPosition(DepositConstants.CLAW_DOWN_POS);
    }

    public void setBusy(boolean busy) {
        isBusy = busy;
    }
}


