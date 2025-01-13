package org.firstinspires.ftc.teamcode.opmode.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.feature.SmartGamepad;
import org.firstinspires.ftc.teamcode.part.Intake;
import org.firstinspires.ftc.teamcode.part.Part;

public class TeleOpMode extends OpMode {
    private Intake intake;

    private Part[] part_list;

    private SmartGamepad smartGamepad1, smartGamepad2;

    @Override
    public void init() {
        smartGamepad1 = new SmartGamepad(gamepad1);
        smartGamepad2 = new SmartGamepad(gamepad2);

        intake = new Intake();

        part_list = new Part[] { intake };

        for (Part part : part_list) {
            part.init(hardwareMap, telemetry);
        }
    }

    @Override
    public void loop() {
        for (Part part : part_list) {
            part.update();
        }
    }

    @Override
    public void stop() {
        for (Part part : part_list) {
            part.stop();
        }
    }
}