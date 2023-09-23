package org.firstinspires.ftc.teamcode.subsystems.drive

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.SubsystemBase

/**
 * A subsystem that uses the [MecanumDrive] class.
 * This periodically calls [MecanumDrive.update] which runs the internal
 * state machine for the mecanum drive. All movement/following is async to fit the paradigm.
 */
class MecanumDriveSubsystem(drive: MecanumDrive, isFieldCentric: Boolean):
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
        val input: Vector2d = Vector2d(-leftY, -leftX)
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

    fun setDrivePower(drivePower: PoseVelocity2d?) {
        drive.setDrivePowers(drivePower)
    }

    fun followAction(action: MecanumDrive.FollowTrajectoryAction?) {

    }

    var poseEstimate: Pose2d
        get() = drive.pose
        set(pose) {
            drive.pose = pose
        }

    fun stop() {
        drive(0.0, 0.0, 0.0)
    }
}