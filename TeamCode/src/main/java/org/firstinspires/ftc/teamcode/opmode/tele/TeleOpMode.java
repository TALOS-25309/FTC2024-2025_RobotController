package org.firstinspires.ftc.teamcode.opmode.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.feature.Schedule;
import org.firstinspires.ftc.teamcode.feature.SmartGamepad;
import org.firstinspires.ftc.teamcode.global.Global;
import org.firstinspires.ftc.teamcode.part.Deposit;
import org.firstinspires.ftc.teamcode.part.Drive;
import org.firstinspires.ftc.teamcode.part.Intake;
import org.firstinspires.ftc.teamcode.part.Part;

@TeleOp(name = "TeleOp")
public class TeleOpMode extends OpMode {
    private Drive drive;
    private Intake intake;
    private Deposit deposit;

    private Part[] part_list;

    private SmartGamepad smartGamepad1, smartGamepad2;

    @Override
    public void init() {
        smartGamepad1 = new SmartGamepad(gamepad1);
        smartGamepad2 = new SmartGamepad(gamepad2);

        drive = new Drive();
        intake = new Intake();
        deposit = new Deposit();

        part_list = new Part[] { drive, intake, deposit };

        for (Part part : part_list) {
            part.init(hardwareMap, telemetry);
        }
    }

    @Override
    public void loop() {
        for (Part part : part_list) {
            part.update();
        }
        Schedule.update();

        checkEmergency();

        if(!Global.IS_EMERGENCY) {
            controlGamepad1();
            Global.ROBOT_STATE = Global.RobotState.INTAKE;
            controlGamepad2();
        }

        smartGamepad1.update();
        Global.ROBOT_STATE = Global.RobotState.INTAKE;
        smartGamepad2.update();

        if (Global.PLAYER1_WARNING)
            smartGamepad1.gamepad().rumble(100);
        if (Global.PLAYER2_WARNING)
            smartGamepad2.gamepad().rumble(100);

        telemetry.addData("STATE",Global.ROBOT_STATE);
        telemetry.update();
    }

    @Override
    public void stop() {
        for (Part part : part_list) {
            part.stop();
        }
    }

    public void controlGamepad1() {
        // Standard Drive
        double x = Math.abs(smartGamepad1.gamepad().left_stick_x) > Math.abs(smartGamepad1.gamepad().right_stick_x)
                ? smartGamepad1.gamepad().left_stick_x
                : smartGamepad1.gamepad().right_stick_x;
        double y = Math.abs(smartGamepad1.gamepad().left_stick_y) > Math.abs(smartGamepad1.gamepad().right_stick_y)
                ? smartGamepad1.gamepad().left_stick_y
                : smartGamepad1.gamepad().right_stick_y;
        double rot = smartGamepad1.gamepad().left_trigger - smartGamepad1.gamepad().right_trigger;
        drive.cmdDrive(x, y, rot);

        // Slow Drive
        x = smartGamepad1.gamepad().dpad_up ? 1 : smartGamepad1.gamepad().dpad_down ? -1 : 0;
        y = smartGamepad1.gamepad().dpad_left ? -1 : smartGamepad1.gamepad().dpad_right ? 1 : 0;
        drive.cmdDriveSlowly(x, y);

        // Road Runner Macro
        if (SmartGamepad.isPressed(smartGamepad1.gamepad().triangle, smartGamepad1.prev().triangle)) {
            drive.cmdAutoAlignBasket();
        } else if (SmartGamepad.isPressed(smartGamepad1.gamepad().cross, smartGamepad1.prev().cross)) {
            drive.cmdAutoAlignSpecimen();
        }

        // Hanging
    }

    public void controlGamepad2() {

        // Horizontal Linear
        if (SmartGamepad.isPressed(smartGamepad2.gamepad().dpad_up, smartGamepad2.prev().dpad_up)) {
            if (Global.ROBOT_STATE == Global.RobotState.NONE) {
                intake.cmdAutoStretch();
            } else {
                Global.PLAYER2_WARNING = true;
            }
        }
        else if (SmartGamepad.isPressed(smartGamepad2.gamepad().dpad_down, smartGamepad2.prev().dpad_down)) {
            if (Global.ROBOT_STATE == Global.RobotState.INTAKE){
                intake.cmdAutoRetract();
                deposit.cmdGrabSample();
            } else {
                Global.PLAYER2_WARNING = true;
            }
        }

        // Auto Intake
        if (Global.ROBOT_STATE == Global.RobotState.INTAKE) {
            if (SmartGamepad.isPressed(smartGamepad2.gamepad().triangle, smartGamepad2.prev().triangle)) {
                intake.cmdIntakeSample();
            } else if (SmartGamepad.isPressed(smartGamepad2.gamepad().circle, smartGamepad2.prev().circle)) {
                intake.cmdIntakeSpecimen();
            } else if (SmartGamepad.isPressed(smartGamepad2.gamepad().square, smartGamepad2.prev().square)) {
                intake.cmdIntakeFreeAngle();
            } else if (SmartGamepad.isPressed(smartGamepad2.gamepad().cross, smartGamepad2.prev().cross)) {
                intake.cmdIntakeVomit();
            }
        }

        // Auto Deposit
        else if (Global.ROBOT_STATE == Global.RobotState.DEPOSIT) {
            if (!deposit.isStretched()) {
                if (SmartGamepad.isPressed(smartGamepad2.gamepad().triangle, smartGamepad2.prev().triangle)) {
                    deposit.cmdDepositSpecimen(Deposit.Location.HIGH);
                } else if (SmartGamepad.isPressed(smartGamepad2.gamepad().circle, smartGamepad2.prev().circle)) {
                    deposit.cmdDepositSpecimen(Deposit.Location.LOW);
                } else if (SmartGamepad.isPressed(smartGamepad2.gamepad().square, smartGamepad2.prev().square)) {
                    deposit.cmdDepositSample(Deposit.Location.HIGH);
                } else if (SmartGamepad.isPressed(smartGamepad2.gamepad().cross, smartGamepad2.prev().cross)) {
                    deposit.cmdDepositSample(Deposit.Location.LOW);
                }
            } else {
                if( SmartGamepad.isPressed(smartGamepad2.gamepad().triangle, smartGamepad2.prev().triangle)
                        || SmartGamepad.isPressed(smartGamepad2.gamepad().circle, smartGamepad2.prev().circle)
                        || SmartGamepad.isPressed(smartGamepad2.gamepad().square, smartGamepad2.prev().square)
                        || SmartGamepad.isPressed(smartGamepad2.gamepad().cross, smartGamepad2.prev().cross)
                ){
                    deposit.cmdReturn();
                }
            }
        } else {
            Global.PLAYER2_WARNING = true;
        }

        // Auto Eater
        if (smartGamepad2.gamepad().left_trigger > 0.5 && smartGamepad2.gamepad().right_trigger > 0.5) {
            intake.cmdAutoRotate();
        }

        // Manual Linear
        if (smartGamepad2.gamepad().left_stick_y < -0.2) {
            intake.cmdManualStretch();
        } else if (smartGamepad2.gamepad().left_stick_y > 0.2) {
            intake.cmdManualRetract();
        } else {
            intake.cmdManualStop();
        }

        if (smartGamepad2.gamepad().right_stick_y < -0.2) {
            deposit.cmdManualStretch();
        } else if (smartGamepad2.gamepad().right_stick_y > 0.2) {
            deposit.cmdManualRetract();
        } else {
            deposit.cmdManualStop();
        }

        // Manual Eater
        if (smartGamepad2.gamepad().left_bumper) {
            intake.cmdManualRotate(-1);
        } else if (smartGamepad2.gamepad().right_bumper) {
            intake.cmdManualRotate(+1);
        }
    }

    public void checkEmergency() {
        if (smartGamepad1.gamepad().left_bumper && smartGamepad1.gamepad().right_bumper) {
            Global.IS_EMERGENCY = true;
            for (Part part : part_list) {
                part.stop();
            }

            smartGamepad1.gamepad().rumble(500);
            smartGamepad2.gamepad().rumble(500);

        } else {
            Global.IS_EMERGENCY = false;
        }
    }
}
