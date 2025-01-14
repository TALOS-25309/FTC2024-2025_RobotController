package org.firstinspires.ftc.teamcode.part;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
class DriveConstants {
    private DriveConstants() {} // Prevent instantiation

    // Motor Speed
    public static final double MOTOR_SPEED = 1.0;
    public static final double MOTOR_SPEED_SLOW = 0.5;

    // X, Y, Omega Weight (Speed)
    public static final double VX_WEIGHT = 1.0;
    public static final double VY_WEIGHT = 1.0;
    public static final double OMEGA_WEIGHT = 1.0;

    // Localization
    public static final boolean USING_LOCALIZATION_BASED_DRIVE = true;
    public static final Pose2d AUTO_INITIAL_POSITION = new Pose2d(0, 0, 0);
    public static final Pose2d TELE_INITIAL_POSITION = new Pose2d(0, 0, 0);
    public static Pose2d INITIAL_POSITION = AUTO_INITIAL_POSITION;

    // Trajectory Values
    public static final Vector2d BASKET_POSITION = new Vector2d(-72, -72);
    public static final double BASKET_RADIUS = 6;
    public static final Vector2d SPECIMEN_POSITION = new Vector2d(0, 0);
}

public class Drive implements Part {
    private Telemetry telemetry;

    private DcMotor leftFront, leftRear, rightRear, rightFront;
    private DcMotor[] motors = {leftFront, leftRear, rightRear, rightFront};

    private SampleMecanumDrive drive;
    private Pose2d robotPose;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize Motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Initialize Drive
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.drive.setPoseEstimate(DriveConstants.INITIAL_POSITION);
    }

    public void update() {
        this.drive.update();

        // Get Current Pose
        this.robotPose = this.drive.getPoseEstimate();
        telemetry.addData("x", robotPose.getX());
        telemetry.addData("y", robotPose.getY());
        telemetry.addData("heading", robotPose.getHeading());

        telemetry.update();
    }

    public void stop() {
        this.drive.breakFollowing();
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return this.drive.trajectoryBuilder(this.robotPose);
    }

    public boolean isBusy() {
        return this.drive.isBusy();
    }

    public void cmdDrive(double x, double y, double omega) {
        if (x != 0.0 || y != 0.0 || omega != 0.0) {
            if (this.drive.isBusy())
                this.drive.breakFollowing();
        }

        if (DriveConstants.USING_LOCALIZATION_BASED_DRIVE) {
            double angle = DriveConstants.INITIAL_POSITION.getHeading() + this.robotPose.getHeading();

            Vector2d rotVector = new Vector2d(x, y).rotated(-angle);
            x = rotVector.getX();
            y = rotVector.getY();
        }

        Pose2d drivePower = new Pose2d (
                x * DriveConstants.VX_WEIGHT,
                y * DriveConstants.VY_WEIGHT,
                omega * DriveConstants.OMEGA_WEIGHT
        );

        double denom = Math.abs(drivePower.getX()) + Math.abs(drivePower.getY()) + Math.abs(drivePower.getHeading());
        if (denom > 1) {
            drivePower = new Pose2d(drivePower.getX(), drivePower.getY(), drivePower.getHeading()).div(denom);
        }

        this.motors[0].setPower(drivePower.getX() - drivePower.getY() - drivePower.getHeading());
        this.motors[1].setPower(drivePower.getX() + drivePower.getY() - drivePower.getHeading());
        this.motors[2].setPower(drivePower.getX() - drivePower.getY() + drivePower.getHeading());
        this.motors[3].setPower(drivePower.getX() + drivePower.getY() + drivePower.getHeading());
    }

    public void cmdDriveSlowly(double x, double y) {
        if (x != 0.0 || y != 0.0) {
            if (this.drive.isBusy())
                this.drive.breakFollowing();
        }

        this.motors[0].setPower(x * DriveConstants.MOTOR_SPEED_SLOW - y * DriveConstants.MOTOR_SPEED_SLOW);
        this.motors[1].setPower(x * DriveConstants.MOTOR_SPEED_SLOW + y * DriveConstants.MOTOR_SPEED_SLOW);
        this.motors[2].setPower(x * DriveConstants.MOTOR_SPEED_SLOW - y * DriveConstants.MOTOR_SPEED_SLOW);
        this.motors[3].setPower(x * DriveConstants.MOTOR_SPEED_SLOW + y * DriveConstants.MOTOR_SPEED_SLOW);
    }

    public void cmdFollowTrajectory(TrajectorySequence trajectory) {
        this.drive.followTrajectorySequence(trajectory);
    }

    public void cmdFollowBasketTrajectory() {
        double dx = DriveConstants.BASKET_POSITION.getX() - this.robotPose.getX();
        double dy = DriveConstants.BASKET_POSITION.getY() - this.robotPose.getY();
        double angle = Math.atan2(dy, dx);

        Vector2d target = new Vector2d(
                DriveConstants.BASKET_POSITION.getX() - Math.cos(angle) * DriveConstants.BASKET_RADIUS,
                DriveConstants.BASKET_POSITION.getY() - Math.sin(angle) * DriveConstants.BASKET_RADIUS);

        TrajectorySequence basketTrajectory = this.drive.trajectorySequenceBuilder(this.robotPose)
                .splineTo(target, angle)
                .build();

        this.drive.followTrajectorySequenceAsync(basketTrajectory);
    }

    public void cmdFollowSpecimenTrajectory() {
        TrajectorySequence specimenTrajectory = this.drive.trajectorySequenceBuilder(this.robotPose)
                .splineTo(DriveConstants.SPECIMEN_POSITION, Math.toRadians(90))
                .build();

        this.drive.followTrajectorySequenceAsync(specimenTrajectory);
    }
}