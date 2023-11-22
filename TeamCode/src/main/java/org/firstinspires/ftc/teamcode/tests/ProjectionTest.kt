package org.firstinspires.ftc.teamcode.tests

import android.util.Log
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.subsystems.cv.AprilTag
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.ProjectionTrajectoryFollowerCommand
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import kotlin.math.PI

@Config
@Autonomous
class ProjectionTest : CommandOpMode() {
    private lateinit var drive: MecanumDriveSubsystem
    val target = 10
    override fun initialize() {
        val cv = AprilTag(hardwareMap, this, telemetry)
        while(cv.getPose(target) == null){}
        val startPos = Pose2d(cv.getPose(target)!!.position+Vector2d(
            AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(target).fieldPosition.get(0).toDouble(),
            AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(target).fieldPosition.get(1).toDouble(),
        ), 0.0)
        Log.d("cvPose", startPos.toString())
        drive = MecanumDriveSubsystem(hardwareMap, startPos, false)

        waitForStart()
        schedule(ProjectionTrajectoryFollowerCommand(drive, TrajectoryBuilder(
            startPos, 1e-6, 0.0,
            drive.drive.defaultVelConstraint, drive.drive.defaultAccelConstraint, 0.25, 0.1
        ).splineTo(Vector2d(
            AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(target).fieldPosition.get(0).toDouble()+5,
            AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(target).fieldPosition.get(1).toDouble(),
        ),0.0).build()[0], target, cv), InstantCommand({cv.visionPortal.close()}))
    }
}