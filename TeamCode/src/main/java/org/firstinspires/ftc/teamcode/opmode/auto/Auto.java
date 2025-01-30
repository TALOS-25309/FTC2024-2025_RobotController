package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.feature.Schedule;
import org.firstinspires.ftc.teamcode.feature.SmartGamepad;
import org.firstinspires.ftc.teamcode.feature.SmartServo;
import org.firstinspires.ftc.teamcode.global.Global;
import org.firstinspires.ftc.teamcode.part.Deposit;
import org.firstinspires.ftc.teamcode.part.Drive;
import org.firstinspires.ftc.teamcode.part.Intake;
import org.firstinspires.ftc.teamcode.part.Part;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous(name = "EasyAuto")
public class Auto extends OpMode {

    private Drive drive;
    private Intake intake;
    private Deposit deposit;

    private Part[] part_list;

    Encoder leftEncoder, rightEncoder;

    @Override
    public void init() {
        Global.ROBOT_STATE = Global.RobotState.DEPOSIT;

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));

        drive = new Drive();
        intake = new Intake();
        deposit = new Deposit();

        part_list = new Part[] { drive, intake, deposit };

        for (Part part : part_list) {
            part.init(hardwareMap, telemetry);
        }

        leftEncoder.setDirection(Encoder.Direction.FORWARD);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    int step = 1;

    int target = 8200;

    @Override
    public void start() {
        deposit.cmdDepositSpecimen(Deposit.Location.HIGH);
    }

    @Override
    public void loop() {
        for (Part part : part_list) {
            part.update();
        }
        Schedule.update();

        int pos = (leftEncoder.getCurrentPosition() + rightEncoder.getCurrentPosition()) / 2;

        telemetry.addData("Pos", pos);

        telemetry.update();

        switch(step) {
            case 1:
                int err = target - pos;
                double value = err * 0.0005;
                if (value > 0.3) value = 0.3;
                if (value < -0.3) value = -0.3;
                drive.cmdDrive(0, value, 0);

                if(err < 10) {
                    step += 1;
                }
                break;
            case 2:
                drive.cmdDrive(0,0,0);
                deposit.cmdReturnSpecimen();
                step += 1;
                break;
        }

        SmartServo.updateAll();
    }

}
