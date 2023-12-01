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
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeDetector
import org.firstinspires.ftc.teamcode.subsystems.drive.Localizer
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.commands.TrajectoryFollowerCommand


@Config
@Autonomous
class AutoTest : CommandOpMode() {
    private lateinit var drive: MecanumDriveSubsystem
    private lateinit var spike: SpikeDetector
    override fun initialize() {
        val startPos = Pose2d(12.0, 62.5, Math.toRadians(90.0))
        drive = MecanumDriveSubsystem(hardwareMap, startPos, false)
        drive.poseEstimate = startPos

        val spikeLeft = drive.drive.actionBuilder(drive.poseEstimate)
            .setReversed(true)
            .strafeToLinearHeading(Vector2d(41.0, 36.0), Math.toRadians(0.0))
            .waitSeconds(1.0)
            .strafeToLinearHeading(Vector2d(47.0, 43.0), Math.toRadians(180.0))
            //CYCLE 1
            .setReversed(false)
            .splineToConstantHeading(Vector2d(24.0, 6.5), Math.toRadians(180.0))
            .strafeToConstantHeading(Vector2d(-56.0, 6.5))
            .waitSeconds(1.0)
            .setReversed(true)
            .strafeToConstantHeading(Vector2d(24.0, 6.5))
            .splineToConstantHeading(Vector2d(48.0, 36.5), Math.toRadians(90.0))
            //CYCLE 2
            .setReversed(false)
            .splineToConstantHeading(Vector2d(24.0, 6.5), Math.toRadians(180.0))
            .strafeToConstantHeading(Vector2d(-56.0, 6.5))
            .waitSeconds(1.0)
            .setReversed(true)
            .strafeToConstantHeading(Vector2d(24.0, 6.5))
            .splineToConstantHeading(Vector2d(48.0, 36.5), Math.toRadians(90.0))
            .build()

        val spikeMiddle = drive.drive.actionBuilder(drive.poseEstimate)
            .setReversed(true)
            .strafeToLinearHeading(Vector2d(19.0, 43.5), Math.toRadians(90.0))
            .waitSeconds(1.0)
            .splineToLinearHeading(Pose2d(47.0, 36.0, Math.toRadians(180.0)), Math.toRadians(270.0))
            .build()

        val spikeRight = drive.drive.actionBuilder(drive.poseEstimate)
            .setReversed(true)
            .strafeToLinearHeading(Vector2d(17.0, 36.0), Math.toRadians(0.0))
            .waitSeconds(1.0)
            .strafeToLinearHeading(Vector2d(47.0, 29.0), Math.toRadians(180.0))
            .build()

        while (opModeInInit()){
            telemetry.addData("Spike Position", spike.position.toString())
            telemetry.update()
        }

        waitForStart()
        spike.visionPortal.close()
        schedule(
            SequentialCommandGroup(
                TrajectoryFollowerCommand(drive, spikeLeft),
            )
        )
    }
}
