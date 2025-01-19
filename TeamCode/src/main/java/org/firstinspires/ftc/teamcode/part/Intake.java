package org.firstinspires.ftc.teamcode.part;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.feature.Schedule;
import org.firstinspires.ftc.teamcode.feature.SmartServo;
import org.firstinspires.ftc.teamcode.global.Global;

import java.net.PortUnreachableException;

@Config
class IntakeConstants {
    private IntakeConstants() {} // Prevent instantiation

    public void asdf() {
        DepositConstants.CLAW_ARM_UP_POS = 5;
    }

    // Horizontal Linear
    public static double HOR_LINEAR_INNER_POSE = 0.0;
    public static double HOR_LINEAR_OUTER_POSE = 0.0;

    public enum HorLinearMode { MANUAL, AUTO, EMERGENCY }
    public static HorLinearMode HOR_LINEAR_MODE = HorLinearMode.AUTO;

    public static double HOR_LINEAR_AUTO_SPEED = 0.3;
    public static double HOR_LINEAR_MANUAL_SPEED = 0.3;

    public static double HOR_LINEAR_kP = 0.2 * 0.0001;

    // Eater
    public static double EATER_ARM_DOWN_POSE_SAMPLE = 0.18;
    public static double EATER_ARM_DOWN_POSE_SPECIMEN = 0.15;
    public static double EATER_ARM_NEUTRAL_POSE = 0.5;
    public static double EATER_ARM_UP_POSE = 0.7;

    public static double EATER_ARM_ANGLE_CONSTANT = -0.035;

    public static double EATER_HAND_DOWN_POSE_SAMPLE = 0.3;
    public static double EATER_HAND_DOWN_POSE_SPECIMEN = 0.5;
    public static double EATER_HAND_NEUTRAL_POSE = 0.15;
    public static double EATER_HAND_UP_POSE = 0.15;

    public static double EATER_ANGLE_UP = 0.51;
    public static double EATER_ANGLE_SPECIMEN = 0.18;
    public static double EATER_ANGLE_SAMPLE = 0.845;

    public static double EATER_SPEED = 0.5;

    public static double EATER_HAND_SPEED = 0.02;

    public static double DELAY_ARM_DOWN_1 = 1;
    public static double DELAY_ARM_DOWN_2 = 1;
    public static double DELAY_ARM_DOWN_3 = 1;
    public static double DELAY_ARM_DOWN_4 = 1;

    public static double DELAY_ARM_UP_1 = 1;
    public static double DELAY_ARM_UP_2 = 1;
    public static double DELAY_ARM_UP_3 = 1;


    // new eater const
        // intake
    public static double EATER_ARM_DOWN_POSE_1 = 0.15; //돌리기 위해서 아래로
    public static double EATER_ARM_DOWN_POSE_2 = 0.06; // 실제로 먹는 위치
    public static double EATER_HAND_DOWN_POSE = 0.2; // 손이 내려간 위치
    public static double EATER_ANGLE_DOWN = 0.18; // 로테이션

        // click
    public static double EATER_CLICK_ARM_SET_POSE = 0.25;
    public static double EATER_CLICK_HAND_SET_POSE = 0.6;

    public static double EATER_CLICK_HAND_ROTATE_ANGLE_VER = 0.51;
    public static double EATER_CLICK_HAND_ROTATE_ANGLE_HOR = 0.85;

    public static double EATER_CLICK_ARM_POSE_1 = 0.2;
    public static double EATER_CLICK_ARM_POSE_2 = 0.3;



    // (both) up
    public static double EATER_ARM_UP_POSE_1 = 0.35;
//    public static double EATER_ANGLE_UP = 0.51;
//    public static double EATER_ARM_UP_POSE
//    public static double EATER_HAND_UP_POSE

    //DELAY
    public static double DELAY_LINEAR_RETRACT = 0;
    public static double DELAY_ARM_UP = 0;
    public static double DELAY_ARM_COMPLETE = 2;
    public static double DELAY_ARM_REST = 5;
    public static double DELAY_ARM_ROTATION_AND_MOVEMENT = 1.0;

    public static double DELAY_INTAKE_DOWN = DELAY_ARM_DOWN_1 + DELAY_ARM_DOWN_2 + DELAY_ARM_DOWN_3 + DELAY_ARM_DOWN_4;
    public static double DELAY_INTAKE_UP = 3;
    public static double DELAY_EATER_RUN = DELAY_ARM_UP_1 + DELAY_ARM_UP_2 + DELAY_ARM_UP_3;

    public static double DELAY_CLICK_SETTING_1 = 1;
    public static double DELAY_CLICK_SETTING_2 = 1;

    public static double DELAY_CLICK_1 = 1;
    public static double DELAY_CLICK_2 = 1;
    public static double DELAY_CLICK_3 = 1;
    public static double DELAY_CLICK_4 = 1;

    public static double DELAY_CLICK_SETTING = DELAY_CLICK_SETTING_1 + DELAY_CLICK_SETTING_2;
    public static double DELAY_CLICK_ROTATE = 3;
    public static double DELAY_CLICK = 3;
    public static double DELAY_CLICK_UP = 3;

}


// Main Part
public class Intake implements Part{
    private Telemetry telemetry;

    private final HorizontalLinear horizontalLinear = new HorizontalLinear();
    private final Eater eater = new Eater();

    private boolean isBusy = false;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        horizontalLinear.init(hardwareMap, telemetry);
        eater.init(hardwareMap, telemetry);
    }

    public void update() {
        if(!Global.IS_TEST)
            horizontalLinear.update();
        eater.update();
    }

    public void stop() {
        horizontalLinear.stop();
        eater.stop();
    }

//    public void cmdIntakeSample() {
//        eater.cmdArmDownForSample();
//        eater.cmdEaterRun(true);
//    }
//    public void cmdIntakeSpecimen() {
//        eater.cmdArmDownForSpecimen();
//        eater.cmdEaterRun(true);
//    }
//    public void cmdIntakeFreeAngle() {
//        eater.cmdArmDownForFreeAngle();
//        eater.cmdEaterRun(true);
//    }

    // 후루룩 먹기 -> 전달
    public void cmdIntake() {
        if(isBusy) return;
        setBusy(true);

        double delay = 0;

        Schedule.addTask(eater::cmdIntakeDown, delay);
        delay += IntakeConstants.DELAY_INTAKE_DOWN;

        Schedule.addTask(()->{eater.cmdEaterRun(true);}, delay);
        delay += IntakeConstants.DELAY_EATER_RUN;

        Schedule.addTask(()->{
            eater.cmdEaterStop();
            eater.cmdIntakeUp();
        }, delay);
        delay += IntakeConstants.DELAY_INTAKE_UP;

        Schedule.addTask(()->{
            setBusy(false);
        },delay);
    }

    // 찍어서 먹기 -> 전달
    public void cmdClick(boolean isVertical) {
        if(isBusy) return;
        setBusy(true);

        double delay = 0;

        Schedule.addTask(eater::cmdClickSetting, delay);
        delay += IntakeConstants.DELAY_CLICK_SETTING;

        Schedule.addTask(()->{
            eater.cmdClickRotate(isVertical);
        } ,delay);
        delay += IntakeConstants.DELAY_CLICK_ROTATE;

        Schedule.addTask(eater::cmdClick, delay);
        delay += IntakeConstants.DELAY_CLICK;

        Schedule.addTask(eater::cmdClickUp, delay);
        delay += IntakeConstants.DELAY_CLICK_UP;

        Schedule.addTask(()->{
            setBusy(false);
        },delay);

    }

    public void cmdIntakeVomit() {
        eater.cmdEaterRun(false);
        eater.cmdArmNeutral();
    }
    public void cmdAutoStretch() {
        Global.ROBOT_STATE = Global.RobotState.INTAKE;

        horizontalLinear.cmdSetMode(IntakeConstants.HorLinearMode.AUTO);
        horizontalLinear.cmdStretch();
    }
    public void cmdAutoRetract() {
        Global.ROBOT_STATE = Global.RobotState.DEPOSIT;
        horizontalLinear.cmdSetMode(IntakeConstants.HorLinearMode.AUTO);

        eater.cmdEaterStop();

        Schedule.addTask(horizontalLinear::cmdRetract, IntakeConstants.DELAY_LINEAR_RETRACT);
        Schedule.addTask(eater::cmdIntakeUp, IntakeConstants.DELAY_ARM_UP);

        Schedule.addTask(eater::cmdArmNeutral, IntakeConstants.DELAY_ARM_REST);
    }
//    public void cmdAutoRotate() {
//        eater.cmdHandAutoRotate();
//    }
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


    public void cmdEaterRun(){
        eater.cmdEaterRun(true);
    }
    public void cmdEaterStop(){
        eater.cmdEaterStop();
    }

    public boolean isBusy(){
        return isBusy;
    }
    public void setBusy(boolean busy) {
        isBusy = busy;
    }
}

// Sub Part
class HorizontalLinear implements Part {
    private Telemetry telemetry;

    private DcMotor motor;
    private double targetPosition = IntakeConstants.HOR_LINEAR_INNER_POSE;
    private boolean isUsingPID = false;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

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
        if(Global.IS_TEST) return;
        if (IntakeConstants.HOR_LINEAR_MODE == IntakeConstants.HorLinearMode.AUTO) {
            this.targetPosition = IntakeConstants.HOR_LINEAR_OUTER_POSE;
            this.isUsingPID = true;
        } else {
            this.isUsingPID = false;
            motor.setPower(IntakeConstants.HOR_LINEAR_MANUAL_SPEED);
        }
    }
    public void cmdRetract() {
        if(Global.IS_TEST) return;
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
    private Telemetry telemetry;

    private SmartServo armServo1, armServo2;
    private SmartServo handServo, handRotationServo;
    private CRServo eaterServo;

    private double targetAngle;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        armServo1 = new SmartServo(hardwareMap.get(Servo.class, "intakeArm1"), "intakeArm1");
        armServo2 = new SmartServo(hardwareMap.get(Servo.class, "intakeArm2"), "intakeArm2");
        handServo = new SmartServo(hardwareMap.get(Servo.class, "intakeHand"), "intakeHand");
        handRotationServo = new SmartServo(hardwareMap.get(Servo.class, "intakeRotation"), "intakeRotation");
        eaterServo = hardwareMap.get(CRServo.class, "intakeEater");

        handServo.servo().setDirection(Servo.Direction.FORWARD);
        handRotationServo.servo().setDirection(Servo.Direction.FORWARD);
        armServo1.servo().setDirection(Servo.Direction.FORWARD);
        armServo2.servo().setDirection(Servo.Direction.REVERSE);

        armServo1.setPosition(IntakeConstants.EATER_ARM_NEUTRAL_POSE + IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
        armServo2.setPosition(IntakeConstants.EATER_ARM_NEUTRAL_POSE - IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
        handServo.setPosition(IntakeConstants.EATER_HAND_NEUTRAL_POSE);
        handRotationServo.setPosition(IntakeConstants.EATER_ANGLE_UP);
    }

    public void update() {

    }

    public void stop() {
        cmdEaterStop();
        cmdHandManualRotate(0);
    }

    public void cmdEaterRun(boolean inside){
        eaterServo.setPower(inside ? IntakeConstants.EATER_SPEED : -IntakeConstants.EATER_SPEED);
    }
    public void cmdEaterStop(){
        eaterServo.setPower(0);
    }
//    public void cmdHandAutoRotate(){
//        Vision.Sample target = Vision.detectTarget();
//        if (target != null){
//            targetAngle = target.angle;
//        }
//        handRotationServo.setPosition(targetAngle);
//    }
    public void cmdHandManualRotate(int direction){
        targetAngle += direction * IntakeConstants.EATER_HAND_SPEED;

        if (targetAngle < 0) targetAngle = 0;
        else if (targetAngle > 1) targetAngle = 1;

        handRotationServo.setPosition(targetAngle);
    }
//    public void cmdArmUp(){
//        targetAngle = IntakeConstants.EATER_ANGLE_UP;
//
//        Schedule.addTask(()->{armServo1.setPosition(IntakeConstants.EATER_ARM_씨발 + IntakeConstants.EATER_ARM_ANGLE_CONSTANT, 1.0);}, IntakeConstants.DELAY_ARM_ROTATION_AND_MOVEMENT * 0);
//        Schedule.addTask(()->{armServo2.setPosition(IntakeConstants.EATER_ARM_씨발 - IntakeConstants.EATER_ARM_ANGLE_CONSTANT, 1.0);}, IntakeConstants.DELAY_ARM_ROTATION_AND_MOVEMENT * 0);
//
//        Schedule.addTask(()->{handServo.setPosition(IntakeConstants.EATER_HAND_DOWN_ROTATION);}, IntakeConstants.DELAY_ARM_ROTATION_AND_MOVEMENT * 1);
//
//        Schedule.addTask(()->{handRotationServo.setPosition(IntakeConstants.EATER_ANGLE_UP);}, IntakeConstants.DELAY_ARM_ROTATION_AND_MOVEMENT * 2);
//
//        Schedule.addTask(()->{armServo1.setPosition(IntakeConstants.EATER_ARM_UP_POSE + IntakeConstants.EATER_ARM_ANGLE_CONSTANT, 1.0);}, IntakeConstants.DELAY_ARM_ROTATION_AND_MOVEMENT * 3);
//        Schedule.addTask(()->{armServo2.setPosition(IntakeConstants.EATER_ARM_UP_POSE - IntakeConstants.EATER_ARM_ANGLE_CONSTANT, 1.0);}, IntakeConstants.DELAY_ARM_ROTATION_AND_MOVEMENT * 3);
//
//        Schedule.addTask(()->{handServo.setPosition(IntakeConstants.EATER_HAND_UP_POSE);}, IntakeConstants.DELAY_ARM_ROTATION_AND_MOVEMENT * 4);
//    }

    public void cmdIntakeDown() {
        double delay = 0;

        Schedule.addTask(() -> {
            armServo1.setPosition(IntakeConstants.EATER_ARM_DOWN_POSE_1 + IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
            armServo2.setPosition(IntakeConstants.EATER_ARM_DOWN_POSE_1 - IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
        }, delay);
        delay += IntakeConstants.DELAY_ARM_DOWN_1;

        Schedule.addTask(()->{
            handServo.setPosition(IntakeConstants.EATER_HAND_DOWN_POSE);
        }, delay);
        delay += IntakeConstants.DELAY_ARM_DOWN_2;

        Schedule.addTask(()->{
            handServo.setPosition(IntakeConstants.EATER_ANGLE_DOWN);
        },delay);
        delay += IntakeConstants.DELAY_ARM_DOWN_3;

        Schedule.addTask(()->{
            armServo1.setPosition(IntakeConstants.EATER_ARM_DOWN_POSE_2 + IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
            armServo2.setPosition(IntakeConstants.EATER_ARM_DOWN_POSE_2 - IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
        }, delay);
        delay += IntakeConstants.DELAY_ARM_DOWN_4;

        Schedule.addTask(()->{}, delay);
    }
    public void cmdIntakeUp() {
        double delay = 0;

        Schedule.addTask(()->{
            armServo1.setPosition(IntakeConstants.EATER_ARM_UP_POSE_1 + IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
            armServo2.setPosition(IntakeConstants.EATER_ARM_UP_POSE_1 - IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
        }, delay);
        delay += IntakeConstants.DELAY_ARM_UP_1;

        Schedule.addTask(()->{handRotationServo.setPosition(IntakeConstants.EATER_ANGLE_UP);}, delay);
        delay += IntakeConstants.DELAY_ARM_UP_2;

        Schedule.addTask(()->{
            armServo1.setPosition(IntakeConstants.EATER_ARM_UP_POSE + IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
            armServo1.setPosition(IntakeConstants.EATER_ARM_UP_POSE - IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
            handServo.setPosition(IntakeConstants.EATER_HAND_UP_POSE);
        }, delay);
        delay += IntakeConstants.DELAY_ARM_UP_3;

        Schedule.addTask(()->{}, delay);
    }

    public void cmdClickSetting() {
        double delay = 0;

        Schedule.addTask(()->{
            armServo1.setPosition(IntakeConstants.EATER_CLICK_ARM_SET_POSE + IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
            armServo2.setPosition(IntakeConstants.EATER_CLICK_ARM_SET_POSE - IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
        }, delay);
        delay += IntakeConstants.DELAY_CLICK_SETTING_1;

        Schedule.addTask(()->{
            handServo.setPosition(IntakeConstants.EATER_CLICK_HAND_SET_POSE);
        }, delay);
        delay += IntakeConstants.DELAY_CLICK_SETTING_2;
    }
    public void cmdClickRotate(boolean isVertical) {
        double angle = isVertical
                ? IntakeConstants.EATER_CLICK_HAND_ROTATE_ANGLE_VER
                : IntakeConstants.EATER_CLICK_HAND_ROTATE_ANGLE_HOR;
        handRotationServo.setPosition(angle);
    }
    public void cmdClick() {
        double delay = 0;

        Schedule.addTask(()->{
            cmdEaterRun(true);
        },delay);
        delay += IntakeConstants.DELAY_CLICK_1;

        Schedule.addTask(()->{
            armServo1.setPosition(IntakeConstants.EATER_CLICK_ARM_POSE_1 + IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
            armServo2.setPosition(IntakeConstants.EATER_CLICK_ARM_POSE_1- IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
        },delay);
        delay += IntakeConstants.DELAY_CLICK_2;

        Schedule.addTask(()->{
            armServo1.setPosition(IntakeConstants.EATER_CLICK_ARM_POSE_2 + IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
            armServo2.setPosition(IntakeConstants.EATER_CLICK_ARM_POSE_2 - IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
        },delay);
        delay += IntakeConstants.DELAY_CLICK_3;

        Schedule.addTask(this::cmdEaterStop, delay);
        delay += IntakeConstants.DELAY_CLICK_4;
    }
    public void cmdClickUp() {
        double delay = 0;

        Schedule.addTask(()->{
            armServo1.setPosition(IntakeConstants.EATER_ARM_UP_POSE_1 + IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
            armServo2.setPosition(IntakeConstants.EATER_ARM_UP_POSE_1 - IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
        }, delay);
        delay += IntakeConstants.DELAY_ARM_UP_1;

        Schedule.addTask(()->{handRotationServo.setPosition(IntakeConstants.EATER_ANGLE_UP);}, delay);
        delay += IntakeConstants.DELAY_ARM_UP_2;

        Schedule.addTask(()->{
            armServo1.setPosition(IntakeConstants.EATER_ARM_UP_POSE + IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
            armServo1.setPosition(IntakeConstants.EATER_ARM_UP_POSE - IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
            handServo.setPosition(IntakeConstants.EATER_HAND_UP_POSE);
        }, delay);
        delay += IntakeConstants.DELAY_ARM_UP_3;

        Schedule.addTask(()->{}, delay);
    }

    public void cmdArmNeutral(){
        targetAngle = IntakeConstants.EATER_ANGLE_UP;
        handRotationServo.setPosition(targetAngle);

        Schedule.addTask(()->{handServo.setPosition(IntakeConstants.EATER_HAND_NEUTRAL_POSE);}, IntakeConstants.DELAY_ARM_ROTATION_AND_MOVEMENT);

        Schedule.addTask(()->{armServo1.setPosition(IntakeConstants.EATER_ARM_NEUTRAL_POSE + IntakeConstants.EATER_ARM_ANGLE_CONSTANT);}, IntakeConstants.DELAY_ARM_ROTATION_AND_MOVEMENT);
        Schedule.addTask(()->{armServo2.setPosition(IntakeConstants.EATER_ARM_NEUTRAL_POSE - IntakeConstants.EATER_ARM_ANGLE_CONSTANT);}, IntakeConstants.DELAY_ARM_ROTATION_AND_MOVEMENT);
    }
    public void cmdArmDownForSample(){
        targetAngle = IntakeConstants.EATER_ANGLE_SAMPLE;
        Schedule.addTask(()->{handRotationServo.setPosition(targetAngle);}, IntakeConstants.DELAY_ARM_ROTATION_AND_MOVEMENT);

        handServo.setPosition(IntakeConstants.EATER_HAND_DOWN_POSE_SAMPLE);

        armServo1.setPosition(IntakeConstants.EATER_ARM_DOWN_POSE_SAMPLE + IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
        armServo2.setPosition(IntakeConstants.EATER_ARM_DOWN_POSE_SAMPLE - IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
    }
    public void cmdArmDownForSpecimen(){
        targetAngle = IntakeConstants.EATER_ANGLE_SPECIMEN;
        Schedule.addTask(()->{handRotationServo.setPosition(targetAngle);}, IntakeConstants.DELAY_ARM_ROTATION_AND_MOVEMENT);

        handServo.setPosition(IntakeConstants.EATER_HAND_DOWN_POSE_SPECIMEN);

        armServo1.setPosition(IntakeConstants.EATER_ARM_DOWN_POSE_SPECIMEN + IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
        armServo2.setPosition(IntakeConstants.EATER_ARM_DOWN_POSE_SPECIMEN - IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
    }
    public void cmdArmDownForFreeAngle(){
        Schedule.addTask(()->{handRotationServo.setPosition(targetAngle);}, IntakeConstants.DELAY_ARM_ROTATION_AND_MOVEMENT);

        handServo.setPosition(IntakeConstants.EATER_HAND_DOWN_POSE_SAMPLE);

        armServo1.setPosition(IntakeConstants.EATER_ARM_DOWN_POSE_SAMPLE + IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
        armServo2.setPosition(IntakeConstants.EATER_ARM_DOWN_POSE_SAMPLE - IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
    }


}