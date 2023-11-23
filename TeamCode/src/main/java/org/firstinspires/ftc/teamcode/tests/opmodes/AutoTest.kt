package org.firstinspires.ftc.teamcode.tests.opmodes

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.*
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.subsystems.drive.Localizer
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.commands.TrajectoryFollowerCommand


@Config
@Autonomous
class AutoTest : CommandOpMode() {
    private lateinit var drive: MecanumDriveSubsystem
    override fun initialize() {
        val startPos = Pose2d(11.0, 62.5, Math.toRadians(90.0))
        drive = MecanumDriveSubsystem(hardwareMap, startPos, false)
        drive.poseEstimate = startPos

        val spike = drive.drive.actionBuilder(drive.poseEstimate)
            .setReversed(true)
            .lineToYLinearHeading(35.5, Math.toRadians(180.0))
            .lineToYLinearHeading(48.0, Math.toRadians(90.0))
            .splineToLinearHeading(Pose2d(48.0, 35.5, Math.toRadians(180.0)), Math.toRadians(270.0))
            .setReversed(false)
            .splineToConstantHeading(Vector2d(24.0, 11.5), Math.toRadians(180.0))
            .strafeToConstantHeading(Vector2d(-60.0, 11.5))
            .setReversed(true)
            .strafeToConstantHeading(Vector2d(24.0, 11.5))
            .splineToConstantHeading(Vector2d(48.0, 35.5), Math.toRadians(90.0))
            .build()

        waitForStart()
        schedule(
            SequentialCommandGroup(
                TrajectoryFollowerCommand(drive, spike),
            )
        )
    }
}
