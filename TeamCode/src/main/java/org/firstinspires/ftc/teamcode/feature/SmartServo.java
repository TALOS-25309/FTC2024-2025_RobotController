package org.firstinspires.ftc.teamcode.feature;

import com.qualcomm.robotcore.hardware.Servo;

public class SmartServo {
    private final Servo servo;
    private double position;

    private double targetPosition;
    private double previousPosition;
    private long startTime;
    private long duration;

    public SmartServo(Servo servo) {
        this.servo = servo;
        this.position = 0.5;
        this.targetPosition = 0.5;
        this.previousPosition = 0.5;
        this.startTime = 0;
        this.duration = 0;
    }

    public void setPosition(double position) {
        this.targetPosition = position;
        this.previousPosition = position;
        this.position = position;
        servo.setPosition(position);
    }

    public void serPosition(double position, double duration) {
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

    public Servo Servo() {
        return servo;
    }

    public void update() {
        if (duration == 0) {
            return;
        }

        double progress = (System.nanoTime() - startTime) / (double) duration;
        if (progress >= 1) {
            position = targetPosition;
            duration = 0;
        } else {
            position = (1 - progress) * this.previousPosition + progress * targetPosition;
        }
        servo.setPosition(position);
    }
}
