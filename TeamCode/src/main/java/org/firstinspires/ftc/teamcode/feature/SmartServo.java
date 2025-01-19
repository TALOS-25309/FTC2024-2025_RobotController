package org.firstinspires.ftc.teamcode.feature;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.Vector;

public class SmartServo {
    private final Servo servo;
    private double position;

    private double targetPosition;
    private double previousPosition;
    private long startTime;
    private long duration;

    private String name;
    private static Vector<SmartServo> smartServos = new Vector<SmartServo>();

    public SmartServo(Servo servo, String name) {
        this.servo = servo;
        this.position = 0.5;
        this.targetPosition = 0.5;
        this.previousPosition = 0.5;
        this.startTime = 0;
        this.duration = 0;

        this.name = name;
        smartServos.add(this);
    }

    public void setPosition(double position) {
        this.targetPosition = position;
        this.previousPosition = position;
        this.position = position;
        servo.setPosition(position);
    }

    public void setPosition(double position, double duration) {
        this.targetPosition = position;
        this.previousPosition = this.position;
        this.startTime = System.nanoTime();
        this.duration = (long) (duration * 1e9);
    }

    public double getPosition() {
        return position;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public Servo servo() {
        return servo;
    }

    public void update() {
        if (duration == 0) {
            return;
        }

        double progress = (System.nanoTime() - startTime) / (double) duration;

        if (progress < 0) progress = 0;
        else if (progress > 1) progress = 1;

        progress = 1 - (1-progress) * (1-progress) * (1-progress);
        if (progress >= 1) {
            position = targetPosition;
            duration = 0;
        } else {
            position = (1 - progress) * this.previousPosition + progress * targetPosition;
        }
        servo.setPosition(position);
    }

    public static SmartServo getServoByName(String name) {
        for (SmartServo smartServo : smartServos) {
            if (smartServo.name.equals(name)) {
                return smartServo;
            }
        }
        return null;
    }

    public static void updateAll() {
        for (SmartServo smartServo : smartServos) {
            smartServo.update();
        }
    }
}
