package org.firstinspires.ftc.teamcode.part;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.global.Global;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
class DriveConstants {
    private DriveConstants() {} // Prevent instantiation

    // Motor Speed
    public static double MOTOR_SPEED = 1.0;
    public static double MOTOR_SPEED_SLOW = 0.5;

    // X, Y, Omega Weight (Speed)
    public static double VX_WEIGHT = 1.0;
    public static double VY_WEIGHT = 1.0;
    public static double OMEGA_WEIGHT = 1.0;

    // Localization
    public static boolean USING_LOCALIZATION_BASED_DRIVE = false;
    public static Pose2d AUTO_INITIAL_POSITION = new Pose2d(0, 0, 0);
    public static Pose2d TELE_INITIAL_POSITION = new Pose2d(0, 0, 0);
    public static Pose2d INITIAL_POSITION = AUTO_INITIAL_POSITION;

    // Trajectory Values
    public static Vector2d BASKET_POSITION = new Vector2d(-72, -72);
    public static double BASKET_RADIUS = 6;
    public static Vector2d SPECIMEN_POSITION = new Vector2d(0, 0);

    public static double directionSign = 1.0;
}

public class Drive implements Part {
    private Telemetry telemetry;

    private SampleMecanumDrive drive;
    private Pose2d robotPose;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        // Initialize Drive
        this.drive = new SampleMecanumDrive(hardwareMap);

        this.drive.setPoseEstimate(DriveConstants.INITIAL_POSITION);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void update() {
        this.drive.update();

        // Get Current Pose
        this.robotPose = this.drive.getPoseEstimate();
        telemetry.addData("x", robotPose.getX());
        telemetry.addData("y", robotPose.getY());
        telemetry.addData("heading", robotPose.getHeading());
    }

    public void stop() {
        this.drive.breakFollowing();
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return this.drive.trajectoryBuilder(this.robotPose);
    }

    public boolean isBusy() {
        return this.drive.isBusy();
    }

    public void cmdDrive(double x, double y, double omega) {
        drive.setWeightedDrivePower(
                new Pose2d(
                        y * DriveConstants.directionSign,
                        -x * DriveConstants.directionSign,
                        -omega
                )
        );
    }

    public void cmdFollowTrajectory(TrajectorySequence trajectory) {
        this.drive.followTrajectorySequence(trajectory);
    }

    public void cmdAutoAlignBasket() {
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

    public void cmdAutoAlignSpecimen() {
        TrajectorySequence specimenTrajectory = this.drive.trajectorySequenceBuilder(this.robotPose)
                .splineTo(DriveConstants.SPECIMEN_POSITION, Math.toRadians(90))
                .build();

        this.drive.followTrajectorySequenceAsync(specimenTrajectory);
    }

    public void cmdAutoAlignSubmersible() {

    }

    public void cmdPositiveDirection() {
        DriveConstants.directionSign = 1.0;
    }

    public void cmdNegativeDirection() {
        DriveConstants.directionSign = -1.0;
    }
}