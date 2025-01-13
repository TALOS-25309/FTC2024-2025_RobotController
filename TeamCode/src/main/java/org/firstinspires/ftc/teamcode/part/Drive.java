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

@Config
class DriveConstants {
    private DriveConstants() {} // Prevent instantiation

    // Motor Speed
    public static final double MOTOR_SPEED = 1.0;

    // X, Y, Omega Weight (Speed)
    public static final double VX_WEIGHT = 1.0;
    public static final double VY_WEIGHT = 1.0;
    public static final double OMEGA_WEIGHT = 1.0;

    // Localization
    public static final boolean USING_LOCALIZATION_BASED_DRIVE = true;
    public static final Pose2d AUTO_INITIAL_POSITION = new Pose2d(0, 0, 0);
    public static final Pose2d TELE_INITIAL_POSITION = new Pose2d(0, 0, 0);
    public static Pose2d INITIAL_POSITION = AUTO_INITIAL_POSITION;
}

public class Drive {
    private Telemetry telemetry;

    private DcMotor leftFront, leftRear, rightRear, rightFront;
    private DcMotor[] motors = {leftFront, leftRear, rightRear, rightFront};

    private SampleMecanumDrive drive;
    private Pose2d robotPose;

    public void init(HardwareMap hardwareMap, Telemetry telemetry, boolean isAuto) {
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
        if (isAuto) {
            DriveConstants.INITIAL_POSITION = DriveConstants.AUTO_INITIAL_POSITION;
        } else {
            DriveConstants.INITIAL_POSITION = DriveConstants.TELE_INITIAL_POSITION;
        }
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

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return this.drive.trajectoryBuilder(startPose);
    }

    public boolean isBusy() {
        return this.drive.isBusy();
    }

    public void cmdDrive(double x, double y, double omega) {
        if (this.drive.isBusy()) return;

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

    public void cmdFollowTrajectory(Trajectory trajectory) {
        this.drive.followTrajectory(trajectory);
    }

    public void cmdFollowTrajectoryAsync(Trajectory trajectory) {
        this.drive.followTrajectoryAsync(trajectory);
    }
}
