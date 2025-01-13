package org.firstinspires.ftc.teamcode.feature;

import com.qualcomm.robotcore.hardware.Gamepad;

public class SmartGamepad {
    private Gamepad gamepadNow, gamepadLast;

    private SmartGamepad() {}

    public SmartGamepad(Gamepad gamepad) {
        gamepadNow = gamepad;
        gamepadLast = gamepad;
    }

    public Gamepad gamepad() {
        return gamepadNow;
    }

    public Gamepad prev() {
        return gamepadLast;
    }

    public void update(Gamepad gamepad) {
        gamepadLast = gamepadNow;
        gamepadNow = gamepad;
    }

    public static boolean isPressed(boolean now, boolean last) {
        return now && !last;
    }

    public static boolean isReleased(boolean now, boolean last) {
        return !now && last;
    }

    public static boolean isHeld(boolean now, boolean last) {
        return now && last;
    }

    public static boolean isFree(boolean now, boolean last) {
        return !now;
    }
}
