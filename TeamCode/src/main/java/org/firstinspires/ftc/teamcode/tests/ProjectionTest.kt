package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.subsystems.cv.AprilTag
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.ProjectionTrajectoryFollowerCommand

@Config
@Autonomous
class ProjectionTest : CommandOpMode() {
    private lateinit var drive: MecanumDriveSubsystem
    override fun initialize() {
        drive = MecanumDriveSubsystem(hardwareMap, Pose2d(0.0, 0.0, 0.0), false)

        val cv = AprilTag(hardwareMap, this)
        var channel = cv.channel

        waitForStart()
        schedule(ProjectionTrajectoryFollowerCommand(drive, TrajectoryBuilder(
            Pose2d(0.0, 0.0, 0.0), 1e-6, 0.0,
            drive.drive.defaultVelConstraint, drive.drive.defaultAccelConstraint, 0.25, 0.1
        ).lineToX(5.0).build()[1], 5, cv))
    }
}