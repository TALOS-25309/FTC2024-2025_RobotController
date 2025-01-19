package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.feature.Schedule;
import org.firstinspires.ftc.teamcode.feature.SmartGamepad;
import org.firstinspires.ftc.teamcode.feature.SmartServo;
import org.firstinspires.ftc.teamcode.global.Global;
import org.firstinspires.ftc.teamcode.part.Deposit;
import org.firstinspires.ftc.teamcode.part.Drive;
import org.firstinspires.ftc.teamcode.part.Intake;
import org.firstinspires.ftc.teamcode.part.Part;

@TeleOp(name = "Test")
public class PIDTest extends OpMode {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private Deposit deposit;

    private Part[] part_list;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        deposit = new Deposit();

        part_list = new Part[] { deposit };

        for (Part part : part_list) {
            part.init(hardwareMap, telemetry);
        }
    }

    @Override
    public void start() {
        this.deposit.cmdPIDTest();
    }

    @Override
    public void loop() {
        for (Part part : part_list) {
            part.update();
        }
        Schedule.update();
        SmartServo.updateAll();
        telemetry.update();
    }

    @Override
    public void stop() {
        for (Part part : part_list) {
            part.stop();
        }
    }
}
