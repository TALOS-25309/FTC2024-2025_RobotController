package org.firstinspires.ftc.teamcode.global;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Global {
    public enum RobotState { NONE, INTAKE, DEPOSIT }
    public static RobotState ROBOT_STATE = RobotState.NONE;

    public static boolean PLAYER1_WARNING = false;
    public static boolean PLAYER2_WARNING = false;

    public static boolean IS_EMERGENCY = false;

    public static boolean IS_TEST = false;
}
