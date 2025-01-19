package org.firstinspires.ftc.teamcode.part;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.feature.Schedule;
import org.firstinspires.ftc.teamcode.feature.SmartServo;

@Config
class Constants {
    enum UpDown {
        UP,
        DOWN
    }

    enum CloseOpen {
        CLOSE,
        OPEN
    }

    enum INTAKE_MOVE {
        UP,
        NEUTRAL,
        DOWN_SPECIMEN,
        DOWN_SAMPLE
    }

    public static CloseOpen DEPOSIT_CLAW = CloseOpen.CLOSE;
    public static UpDown DEPOSIT_ARM = UpDown.UP;
    public static UpDown DEPOSIT_HAND = UpDown.UP;

    public static INTAKE_MOVE INTAKE_ARM = INTAKE_MOVE.UP;
    public static INTAKE_MOVE INTAKE_HAND = INTAKE_MOVE.UP;
    public static INTAKE_MOVE INTAKE_ANGLE = INTAKE_MOVE.UP;
}

@TeleOp(name = "TeleOp")
public class ServoTestOpMode extends OpMode {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private Intake intake;
    private Deposit deposit;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        intake = new Intake();
        deposit = new Deposit();

        for (Part part : new Part[] { intake, deposit }) {
            part.init(hardwareMap, telemetry);
        }
    }

    @Override
    public void start() {
        if (Constants.DEPOSIT_ARM == Constants.UpDown.UP) {
            SmartServo.getServoByName("depositArm1").setPosition(DepositConstants.CLAW_ARM_UP_POS);
            SmartServo.getServoByName("depositArm2").setPosition(DepositConstants.CLAW_ARM_UP_POS);
        } else if (Constants.DEPOSIT_ARM == Constants.UpDown.DOWN) {
            SmartServo.getServoByName("depositArm1").setPosition(DepositConstants.CLAW_ARM_DOWN_POS);
            SmartServo.getServoByName("depositArm2").setPosition(DepositConstants.CLAW_ARM_DOWN_POS);
        }

        if (Constants.DEPOSIT_HAND == Constants.UpDown.UP) {
            SmartServo.getServoByName("depositHand").setPosition(DepositConstants.CLAW_HAND_UP_POS);
        } else if (Constants.DEPOSIT_HAND == Constants.UpDown.DOWN) {
            SmartServo.getServoByName("depositHand").setPosition(DepositConstants.CLAW_HAND_DOWN_POS);
        }

        if (Constants.DEPOSIT_CLAW == Constants.CloseOpen.CLOSE) {
            SmartServo.getServoByName("depositClaw").setPosition(DepositConstants.CLAW_CLAW_CLOSED_POS);
        } else if (Constants.DEPOSIT_CLAW == Constants.CloseOpen.OPEN) {
            SmartServo.getServoByName("depositClaw").setPosition(DepositConstants.CLAW_CLAW_OPEN_POS);
        }

        if (Constants.INTAKE_ARM == Constants.INTAKE_MOVE.UP) {
            SmartServo.getServoByName("intakeArm1").setPosition(IntakeConstants.EATER_ARM_UP_POSE + IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
            SmartServo.getServoByName("intakeArm2").setPosition(IntakeConstants.EATER_ARM_UP_POSE - IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
        } else if (Constants.INTAKE_ARM == Constants.INTAKE_MOVE.DOWN_SAMPLE) {
            SmartServo.getServoByName("intakeArm1").setPosition(IntakeConstants.EATER_ARM_DOWN_POSE_SAMPLE + IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
            SmartServo.getServoByName("intakeArm2").setPosition(IntakeConstants.EATER_ARM_DOWN_POSE_SAMPLE - IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
        } else if (Constants.INTAKE_ARM == Constants.INTAKE_MOVE.DOWN_SPECIMEN) {
            SmartServo.getServoByName("intakeArm1").setPosition(IntakeConstants.EATER_ARM_DOWN_POSE_SPECIMEN + IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
            SmartServo.getServoByName("intakeArm2").setPosition(IntakeConstants.EATER_ARM_DOWN_POSE_SPECIMEN - IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
        } else if (Constants.INTAKE_ARM == Constants.INTAKE_MOVE.NEUTRAL) {
            SmartServo.getServoByName("intakeArm1").setPosition(IntakeConstants.EATER_ARM_NEUTRAL_POSE + IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
            SmartServo.getServoByName("intakeArm2").setPosition(IntakeConstants.EATER_ARM_NEUTRAL_POSE - IntakeConstants.EATER_ARM_ANGLE_CONSTANT);
        }

        if (Constants.INTAKE_HAND == Constants.INTAKE_MOVE.UP) {
            SmartServo.getServoByName("intakeHand").setPosition(IntakeConstants.EATER_HAND_UP_POSE);
        } else if (Constants.INTAKE_HAND == Constants.INTAKE_MOVE.DOWN_SAMPLE) {
            SmartServo.getServoByName("intakeHand").setPosition(IntakeConstants.EATER_HAND_DOWN_POSE_SAMPLE);
        } else if (Constants.INTAKE_HAND == Constants.INTAKE_MOVE.DOWN_SPECIMEN) {
            SmartServo.getServoByName("intakeHand").setPosition(IntakeConstants.EATER_HAND_DOWN_POSE_SPECIMEN);
        } else if (Constants.INTAKE_HAND == Constants.INTAKE_MOVE.NEUTRAL) {
            SmartServo.getServoByName("intakeHand").setPosition(IntakeConstants.EATER_HAND_NEUTRAL_POSE);
        }

        if (Constants.INTAKE_ANGLE == Constants.INTAKE_MOVE.UP) {
            SmartServo.getServoByName("intakeRotation").setPosition(IntakeConstants.EATER_ANGLE_UP);
        } else if (Constants.INTAKE_ANGLE == Constants.INTAKE_MOVE.DOWN_SAMPLE) {
            SmartServo.getServoByName("intakeRotation").setPosition(IntakeConstants.EATER_ANGLE_SAMPLE);
        } else if (Constants.INTAKE_ANGLE == Constants.INTAKE_MOVE.DOWN_SPECIMEN) {
            SmartServo.getServoByName("intakeRotation").setPosition(IntakeConstants.EATER_ANGLE_SPECIMEN);
        }
    }

    @Override
    public void loop() {
        Schedule.update();
        telemetry.update();

        SmartServo.updateAll();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
