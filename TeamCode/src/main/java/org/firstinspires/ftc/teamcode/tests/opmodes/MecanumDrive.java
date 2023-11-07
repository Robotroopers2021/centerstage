//package org.firstinspires.ftc.teamcode.tests.opmodes;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.localization.Localizer;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//
//import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequenceBuilder;
//
//import java.util.List;
//
///**
// * A subsystem that uses the {@link SampleMecanumDrive} class.
// * This periodically calls {@link SampleMecanumDrive#update()} which runs the internal
// * state machine for the mecanum drive. All movement/following is async to fit the paradigm.
// */
//public class MecanumDrive extends SubsystemBase {
//
//    public final SampleMecanumDrive drive;
//
//    public MecanumDrive(HardwareMap hardwareMap) {
//        this.drive = new SampleMecanumDrive(hardwareMap);
//    }
//
//    public void setMode(DcMotor.RunMode mode) {
//        drive.setMode(mode);
//    }
//
//    public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients coefficients) {
//        drive.setPIDFCoefficients(mode, coefficients);
//    }
//
//    public void setPoseEstimate(Pose2d pose) {
//        drive.setPoseEstimate(pose);
//    }
//
//    public void update() {
//        this.drive.update();
//    }
//
//    public void updatePoseEstimate() {
//        drive.updatePoseEstimate();
//    }
//
//
//    public void setDrivePower(Pose2d drivePower) {
//        drive.setDrivePower(drivePower);
//    }
//
//    public Pose2d getPoseEstimate() {
//        return drive.getPoseEstimate();
//    }
//
//    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
//        return drive.trajectoryBuilder(startPose);
//    }
//
//    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
//        return drive.trajectoryBuilder(startPose, reversed);
//    }
//
//    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
//        return drive.trajectoryBuilder(startPose, startHeading);
//    }
//
//    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose){
//        return drive.trajectorySequenceBuilder(startPose);
//    }
//
//    public void followTrajectory(Trajectory trajectory) {
//        drive.followTrajectoryAsync(trajectory);
//    }
//
//    public void followTrajectorySequence(TrajectorySequence trajectorySequence){
//        drive.followTrajectorySequence(trajectorySequence);
//    }
//
//    public void setWeightedDrivePower(Pose2d drivePower){
//        drive.setWeightedDrivePower(drivePower);
//    }
//
//    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence){
//        drive.followTrajectorySequenceAsync(trajectorySequence);
//    }
//
//    public void drive(double leftY, double leftX, double rightX) {
//        Pose2d poseEstimate = getPoseEstimate();
//
//        Vector2d input = new Vector2d(-leftY, -leftX).rotated(
//                -poseEstimate.getHeading()
//        );
//
//        drive.setWeightedDrivePower(
//                new Pose2d(
//                        input.getX(),
//                        input.getY(),
//                        -rightX
//                )
//        );
//    }
//
//    public boolean isBusy() {
//        return drive.isBusy();
//    }
//
//    public void turn(double radians) {
//        drive.turnAsync(radians);
//    }
//
//    public void stop() {
//        drive.setWeightedDrivePower(new Pose2d(0.0, 0.0, 0.0));
//    }
//
//    public Pose2d getPoseVelocity() {
//        return drive.getPoseVelocity();
//    }
//
//    public Localizer getLocalizer() {
//        return drive.getLocalizer();
//    }
//
//}