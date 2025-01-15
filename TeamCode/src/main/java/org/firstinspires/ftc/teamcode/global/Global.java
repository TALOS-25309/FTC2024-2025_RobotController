package org.firstinspires.ftc.teamcode.global;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Global {
    public enum RobotState { NONE, INTAKE, DEPOSIT }
    public static RobotState robotState = RobotState.NONE;
}
