package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;

import java.util.List;

public class MecanumDriveSubsystem extends SubsystemBase {

    final MecanumDrive drive;
    private final boolean fieldCentric;

    public MecanumDriveSubsystem(MecanumDrive drive, boolean isFieldCentric) {
        this.drive = drive;
        fieldCentric = isFieldCentric;
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.pose = pose;
    }

    public PoseVelocity2d updatePoseEstimate() {
        return drive.updatePoseEstimate();
    }

    public void drive(double leftY, double leftX, double rightX) {
        Pose2d poseEstimate = getPoseEstimate();

        Vector2d input = new Vector2d(-leftY, -leftX);

        drive.setDrivePowers(
                new PoseVelocity2d(
                        input,
                        -rightX
                )
        );
    }

    public void setDrivePower(PoseVelocity2d drivePower) {
        drive.setDrivePowers(drivePower);
    }

    public Pose2d getPoseEstimate() {
        return drive.pose;
    }

    public TrajectoryActionBuilder trajectoryActionBuilder(Pose2d startPose) {
        return drive.actionBuilder(startPose);
    }

    public void followAction(Action action) {
        Actions.runBlocking(action);
    }

    public boolean isBusy() {
        return drive.leftBack.isBusy();
    }

    public void turn(double radians) {
        Actions.runBlocking(drive.actionBuilder(getPoseEstimate())
                .turn(radians)
                .build());
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public Localizer getLocalizer() {
        return drive.localizer;
    }

}