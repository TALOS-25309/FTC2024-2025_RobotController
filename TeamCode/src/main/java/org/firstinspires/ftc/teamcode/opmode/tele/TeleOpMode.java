package org.firstinspires.ftc.teamcode.opmode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.feature.Schedule;
import org.firstinspires.ftc.teamcode.feature.SmartGamepad;
import org.firstinspires.ftc.teamcode.feature.SmartServo;
import org.firstinspires.ftc.teamcode.global.Global;
import org.firstinspires.ftc.teamcode.part.Deposit;
import org.firstinspires.ftc.teamcode.part.Drive;
import org.firstinspires.ftc.teamcode.part.Intake;
import org.firstinspires.ftc.teamcode.part.Part;

@Config
class FUCK {
    public static double FUCKING_VALUE = -0.7;
    public static double SHIT = -0.2;
}

@TeleOp(name = "TeleOp")
public class TeleOpMode extends OpMode {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private Drive drive;
    private Intake intake;
    private Deposit deposit;

    private Part[] part_list;

    private SmartGamepad smartGamepad1, smartGamepad2;

    @Override
    public void init() {
        Global.ROBOT_STATE = Global.RobotState.DEPOSIT;

        telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

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
            controlGamepad2();
        }

        smartGamepad1.update();
        smartGamepad2.update();

        telemetry.addData("WARNING",Global.PLAYER2_WARNING);

        if (Global.PLAYER1_WARNING) {
            smartGamepad1.gamepad().rumble(100);
            Global.PLAYER1_WARNING = false;
        }
        if (Global.PLAYER2_WARNING) {
            smartGamepad2.gamepad().rumble(100);
            Global.PLAYER2_WARNING = false;
        }

        SmartServo.updateAll();

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

        rot += x * (FUCK.FUCKING_VALUE + (x > 0 ? FUCK.SHIT : - FUCK.SHIT));
        drive.cmdDrive(-x, -y, rot);

        // Road Runner Macro
        /*
        if (SmartGamepad.isPressed(smartGamepad1.gamepad().triangle, smartGamepad1.prev().triangle)) {
            drive.cmdAutoAlignBasket();
        } else if (SmartGamepad.isPressed(smartGamepad1.gamepad().cross, smartGamepad1.prev().cross)) {
            drive.cmdAutoAlignSpecimen();
        }
        */

        if (SmartGamepad.isPressed(smartGamepad1.gamepad().circle, smartGamepad1.prev().circle)) {
            drive.cmdChangeDirection();
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
                telemetry.addLine("INTAKE 트랜스퍼");
                intake.cmdMoveUp();
                intake.cmdAutoRetract();
                deposit.cmdGrabSample();
            } else {
                Global.PLAYER2_WARNING = true;
            }
        }

        // Auto Intake
        if (Global.ROBOT_STATE == Global.RobotState.INTAKE) {
            if (SmartGamepad.isPressed(smartGamepad2.gamepad().triangle, smartGamepad2.prev().triangle)) {
                telemetry.addLine("INTAKE 후루룩");
                intake.cmdIntake(); // 후루룩
            } else if (SmartGamepad.isPressed(smartGamepad2.gamepad().circle, smartGamepad2.prev().circle)) {
                telemetry.addLine("세로 집기");
                intake.cmdClick(true); // 세로 찝기
            } else if (SmartGamepad.isPressed(smartGamepad2.gamepad().square, smartGamepad2.prev().square)) {
                telemetry.addLine("가로 집기");
                intake.cmdClick(false); // 가로 찝기
            } else if (SmartGamepad.isPressed(smartGamepad2.gamepad().cross, smartGamepad2.prev().cross)) {
                intake.cmdIntakeVomit(); // 뱉기
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
                    if(deposit.isSample())
                        deposit.cmdReturnSample();
                    else
                        deposit.cmdReturnSpecimen();
                }
            }
        }


        // Manual Hor Linear
        if (smartGamepad2.gamepad().left_stick_y < -0.2) {
            intake.cmdManualStretch();
        } else if (smartGamepad2.gamepad().left_stick_y > 0.2) {
            intake.cmdManualRetract();
        } else {
            intake.cmdManualStop();
        }

        // Manual Ver Linear
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

        // Auto Eater
//        if (smartGamepad2.gamepad().left_trigger > 0.5 && smartGamepad2.gamepad().right_trigger > 0.5) {
//            if (Global.ROBOT_STATE == Global.RobotState.INTAKE)
//                intake.cmdAutoRotate();
//            else
//                Global.PLAYER2_WARNING = true;
//        }
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
