package org.firstinspires.ftc.teamcode.opmode.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.feature.Schedule;
import org.firstinspires.ftc.teamcode.feature.SmartGamepad;
import org.firstinspires.ftc.teamcode.part.Drive;
import org.firstinspires.ftc.teamcode.part.Intake;
import org.firstinspires.ftc.teamcode.part.Part;

public class TeleOpMode extends OpMode {
    private Drive drive;
    private Intake intake;

    private Part[] part_list;

    private SmartGamepad smartGamepad1, smartGamepad2;

    @Override
    public void init() {
        smartGamepad1 = new SmartGamepad(gamepad1);
        smartGamepad2 = new SmartGamepad(gamepad2);

        drive = new Drive();
        intake = new Intake();

        part_list = new Part[] { drive, intake };

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

        contolGamepad1();
        controlGamepad2();

        checkEmergency();
    }

    @Override
    public void stop() {
        for (Part part : part_list) {
            part.stop();
        }
    }

    public void contolGamepad1() {
        // Standard Drive
        double x = Math.max(smartGamepad1.gamepad().left_stick_x, smartGamepad1.gamepad().right_stick_x);
        double y = Math.max(smartGamepad1.gamepad().left_stick_y, smartGamepad1.gamepad().right_stick_y);
        double rot = smartGamepad1.gamepad().left_trigger - smartGamepad1.gamepad().right_trigger;
        drive.cmdDrive(x, y, rot);

        // Slow Drive
        x = smartGamepad1.gamepad().dpad_up ? 1 : smartGamepad1.gamepad().dpad_down ? -1 : 0;
        y = smartGamepad1.gamepad().dpad_left ? -1 : smartGamepad1.gamepad().dpad_right ? 1 : 0;
        drive.cmdDriveSlowly(x, y);

        // Road Runner Macro
        if (SmartGamepad.isPressed(smartGamepad1.gamepad().triangle, smartGamepad1.prev().triangle)) {
            drive.cmdFollowBasketTrajectory();
        } else if (SmartGamepad.isPressed(smartGamepad1.gamepad().circle, smartGamepad1.prev().circle)) {
            drive.cmdFollowSpecimenTrajectory();
        }

        // Hanging
    }

    public void controlGamepad2() {
        // Intake
        if (SmartGamepad.isPressed(smartGamepad2.gamepad().dpad_up, smartGamepad2.prev().dpad_up)) {
            intake.runNextStep();
        }
        else if (SmartGamepad.isPressed(smartGamepad2.gamepad().dpad_down, smartGamepad2.prev().dpad_down)) {
            intake.runPrevStep();
        }

        // Deposit

        // Manual Linear
        if (smartGamepad2.gamepad().left_stick_y > 0.2) {
            Intake.cmdManualStretch();
        } else if (smartGamepad2.gamepad().left_stick_y < -0.2) {
            Intake.cmdManualRetract();
        } else {
            Intake.cmdManualStop();
        }

        // Manual Eater
        if (smartGamepad2.gamepad().left_bumper) {
            Intake.cmdManualRotate(-1);
        } else if (smartGamepad2.gamepad().right_bumper) {
            Intake.cmdManualRotate(+1);
        }

        // Auto Eater
        if (smartGamepad2.gamepad().left_trigger > 0.5 && smartGamepad2.gamepad().right_trigger > 0.5) {
            Intake.cmdAutoRotate();
        }
    }

    public void checkEmergency() {

    }
}
