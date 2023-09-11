package org.firstinspires.ftc.teamcode.subsystems.drive

import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.arcrobotics.ftclib.command.SubsystemBase

/**
 * A subsystem that uses the [MecanumDrive] class.
 * This periodically calls [MecanumDrive.update] which runs the internal
 * state machine for the mecanum drive. All movement/following is async to fit the paradigm.
 */
class MecanumDriveSubsystem(drive: MecanumDrive, isFieldCentric: Boolean) :
    SubsystemBase() {
    private val drive: MecanumDrive
    private val fieldCentric: Boolean

    init {
        this.drive = drive
        fieldCentric = isFieldCentric
    }

    fun updatePoseEstimate() {
        drive.updatePoseEstimate()
    }

    fun drive(leftY: Double, leftX: Double, rightX: Double) {
        val poseEstimate = poseEstimate
        val input: Vector2d = Vector2d(-leftY, -leftX).rotated(
            if (fieldCentric) -poseEstimate.getHeading() else 0
        )
        drive.setDrivePowers(
            PoseVelocity2d(
                Vector2d(
                    -leftY,
                    -leftX
                ),
                -rightX
            )
        )
    }

    fun setDrivePower(drivePower: Pose2d?) {
        drive.setDrivePower(drivePower)
    }

    var poseEstimate: Pose2d
        get() = drive.pose
        set(pose) {
            drive.pose = pose
        }

    fun trajectoryBuilder(startPose: Pose2d?): TrajectoryBuilder {
        return drive.trajectoryBuilder(startPose)
    }

    fun trajectoryBuilder(startPose: Pose2d?, reversed: Boolean): TrajectoryBuilder {
        return drive.trajectoryBuilder(startPose, reversed)
    }

    fun trajectoryBuilder(startPose: Pose2d?, startHeading: Double): TrajectoryBuilder {
        return drive.trajectoryBuilder(startPose, startHeading)
    }

    fun followTrajectory(trajectory: Trajectory?) {
        drive.followTrajectoryAsync(trajectory)
    }

    val isBusy: Boolean
        get() = drive.isBusy()

    fun turn(radians: Double) {
        drive.turnAsync(radians)
    }

    val wheelVelocities: List<Double>
        get() = drive.getWheelVelocities()

    fun stop() {
        drive(0.0, 0.0, 0.0)
    }

    val poseVelocity: Pose2d
        get() = drive.getPoseVelocity()
    val localizer: Localizer
        get() = drive.getLocalizer()
}