package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

 import org.firstinspires.ftc.teamcode.feature.Schedule;
import org.firstinspires.ftc.teamcode.feature.SmartGamepad;
import org.firstinspires.ftc.teamcode.global.Global;
import org.firstinspires.ftc.teamcode.part.Deposit;
import org.firstinspires.ftc.teamcode.part.Drive;
import org.firstinspires.ftc.teamcode.part.Intake;
import org.firstinspires.ftc.teamcode.part.Part;

@Autonomous(name = "Basket")
public class BasketAuto extends LinearOpMode {

    private Drive drive;
    private Intake intake;
    private Deposit deposit;

    private Part[] part_list;


    @Override
    public void runOpMode() {
        waitForStart();

        int delay = 0;


        // 챔버 위치로 이동
        // 팔 올리기




//        Schedule.addTask();
//        ....
//

        while (true){
            drive.update();
            intake.update();
            deposit.update();
        }
    }




}
