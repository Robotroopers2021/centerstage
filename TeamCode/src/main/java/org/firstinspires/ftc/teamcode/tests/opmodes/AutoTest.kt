//package org.firstinspires.ftc.teamcode.tests.opmodes
//
//import android.util.Log
//import com.acmerobotics.dashboard.FtcDashboard
//import com.acmerobotics.dashboard.canvas.Canvas
//import com.acmerobotics.dashboard.config.Config
//import com.acmerobotics.roadrunner.geometry.Pose2d
//import com.acmerobotics.roadrunner.geometry.Vector2d
//import com.arcrobotics.ftclib.command.CommandOpMode
//import com.arcrobotics.ftclib.command.InstantCommand
//import com.arcrobotics.ftclib.command.RunCommand
//import com.arcrobotics.ftclib.command.ScheduleCommand
//import com.arcrobotics.ftclib.command.SequentialCommandGroup
//import com.arcrobotics.ftclib.command.WaitCommand
//import com.qualcomm.hardware.ams.AMSColorSensor.Wait
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous
//import kotlinx.coroutines.runBlocking
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
//import org.firstinspires.ftc.teamcode.subsystems.cv.AprilTag
//import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
//import org.jetbrains.annotations.Async.Schedule
//import kotlin.math.PI
//import kotlin.math.cos
//import kotlin.math.sin
//
//@Config
//@Autonomous
//class AutoTest : CommandOpMode() {
//    private lateinit var drive: MecanumDrive
//    override fun initialize() {
//        drive = MecanumDrive(hardwareMap)
//
//        val startPos = Pose2d(11.0, 62.5, Math.toRadians(90.0))
//        /*telemetry.clear()
//        telemetry.addData("x", drive.poseEstimate.x)
//        telemetry.addData("y", drive.poseEstimate.y)
//        telemetry.addData("heading", Math.toDegrees(drive.poseEstimate.heading))
//        telemetry.update()*/
//
//        drive.poseEstimate = startPos
//        /*telemetry.addData("x", drive.poseEstimate.x)
//        telemetry.addData("y", drive.poseEstimate.y)
//        telemetry.addData("heading", Math.toDegrees(drive.poseEstimate.heading))
//        telemetry.update()*/
//
//        var x = ""
//        var y = ""
//        val cv = AprilTag(hardwareMap, this)
//        var channel = cv.channel
//        var yDist: Double?
//        var detection: Map<Int, Pose2d>
//        var poseEstimate = Pose2d(drive.poseEstimate.x, drive.poseEstimate.y, drive.poseEstimate.heading)
//        var lock = false
//        var targetTagId = 2
//        var tag: Pose2d
//        var targetTag = AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(targetTagId)
//
//        var targetTagPos = Pose2d(
//            Vector2d(
//                targetTag.fieldPosition.get(0).toDouble(),
//                targetTag.fieldPosition.get(1).toDouble()
//            ), targetTag.fieldOrientation.toOrientation(
//            AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle.toDouble())
//
//        val spike =
//            drive.trajectorySequenceBuilder(startPos)
//                .setReversed(true)
//                .lineToLinearHeading(Pose2d(11.0, 34.0, Math.toRadians(180.0)))
//                .lineToLinearHeading(Pose2d(48.0, 34.0, Math.toRadians(180.0)))
//                .build()
//
//        var second =
//            drive.trajectorySequenceBuilder(spike.end())
//                .setReversed(false)
//                .splineTo(Vector2d(0.0, 33.5), Math.toRadians(180.0))
//                .splineTo(Vector2d(-59.0, 33.0), Math.toRadians(180.0))
//                .setReversed(true)
//                .splineTo(Vector2d(0.0, 33.5), Math.toRadians(0.0))
//                .splineTo(Vector2d(48.0, 33.0), Math.toRadians(0.0))
//                .setReversed(false)
////                .splineTo(Vector2d(0.0, 38.0), Math.toRadians(180.0))
////                .splineTo(Vector2d(-60.0, 35.5), Math.toRadians(180.0))
////                .setReversed(true)
////                .splineTo(Vector2d(0.0, 36.0), Math.toRadians(0.0))
////                .splineTo(Vector2d(48.0, 35.5), Math.toRadians(0.0))
////                .setReversed(false)
////                .splineTo(Vector2d(0.0, 38.0), Math.toRadians(180.0))
////                .splineTo(Vector2d(-60.0, 35.5), Math.toRadians(180.0))
////                .setReversed(true)
////                .splineTo(Vector2d(0.0, 36.0), Math.toRadians(0.0))
////                .splineTo(Vector2d(48.0, 35.5), Math.toRadians(0.0))
////                .setReversed(false)
////                .splineTo(Vector2d(0.0, 38.0), Math.toRadians(180.0))
////                .splineTo(Vector2d(-60.0, 35.5), Math.toRadians(180.0))
////                .setReversed(true)
////                .splineTo(Vector2d(0.0, 36.0), Math.toRadians(0.0))
////                .splineTo(Vector2d(48.0, 35.5), Math.toRadians(0.0))
//                .build()
//
//        val goToDeposit =
//            drive.trajectorySequenceBuilder(Pose2d(11.0, 34.5, Math.toRadians(180.0)))
//                .setReversed(true)
//                .lineTo(Vector2d(48.0, 33.0))
//                .build()
//
//        val goToStack =
//            drive.trajectorySequenceBuilder(goToDeposit.end())
//                .setReversed(false)
//                .splineTo(Vector2d(0.0, 33.5), Math.toRadians(180.0))
//                .splineTo(Vector2d(-59.0, 33.0), Math.toRadians(180.0))
//                .build()
//
//        val goToCycle =
//            drive.trajectorySequenceBuilder(goToStack.end())
//                .setReversed(true)
//                .splineTo(Vector2d(0.0, 33.5), Math.toRadians(0.0))
//                .splineTo(Vector2d(48.0, 33.0), Math.toRadians(0.0))
//                .build()
//
//        val dashboard = FtcDashboard.getInstance()
//        val robotRadius = 7.25
//
//        drive.poseEstimate = spike.start()
//        waitForStart()
//        schedule(
//            /*RunCommand({
//                val packet = TelemetryPacket()
//                val field: Canvas = packet.fieldOverlay()
//                field.strokeCircle(drive.poseEstimate.x, drive.poseEstimate.y, robotRadius)
//                val arrowX: Double = cos(drive.poseEstimate.heading) * robotRadius
//                val arrowY: Double = sin(drive.poseEstimate.heading) * robotRadius
//                val x1: Double = drive.poseEstimate.x + arrowX / 2
//                val y1: Double = drive.poseEstimate.y + arrowY / 2
//                val x2: Double = drive.poseEstimate.x + arrowX
//                val y2: Double = drive.poseEstimate.y + arrowY
//                field.strokeLine(x1, y1, x2, y2)
//                dashboard.sendTelemetryPacket(packet)
//                Log.d("RUNCOMMAND", drive.poseEstimate.toString())
//                }),*/
//            SequentialCommandGroup(
//                FollowTrajectorySequence(drive, spike),
//                InstantCommand ({
//                    Log.d("AUTOTEST", "STARTING")
//                    detection = buildMap {cv.aprilTag.detections.forEach {
//                        put(it.id, Pose2d(it.ftcPose.x, -it.ftcPose.y, it.ftcPose.yaw))
//                    }}
//                    if(detection.containsKey(targetTagId)) {
//                        tag = detection[targetTagId]!!
//                        poseEstimate = Pose2d(
//                            targetTagPos.vec() + Vector2d(
//                                tag.y,
//                                -tag.x
//                            ),
//                            (targetTagPos.heading + tag.heading)
//                        ).plus(Pose2d(Vector2d(-4.375, 0.25), 3*PI/2))
//                    }
//                    Log.d("AUTOTEST", detection.size.toString())
//                    Log.d("AUTOTEST", "Tag Pose Estimate: " + Pose2d(poseEstimate.x, poseEstimate.y, poseEstimate.heading).toString())
//                    Log.d("AUTOTEST", "Odo Pose Estimate: " + drive.poseEstimate.toString())
//                    schedule(FollowTrajectorySequence(drive,
//                        drive.trajectorySequenceBuilder(spike.end())
//                            .setReversed(true)
//                            .lineTo(Vector2d(poseEstimate.x, poseEstimate.y))
//                            .setReversed(false)
//                            .splineTo(Vector2d(0.0, 34.5), Math.toRadians(180.0))
//                            .splineTo(Vector2d(-59.0, 34.0), Math.toRadians(180.0))
//                            .setReversed(true)
//                            .splineTo(Vector2d(0.0, 36.0), Math.toRadians(0.0))
//                            .splineTo(Vector2d(48.0, 36.0), Math.toRadians(0.0))
//                            .build()
//                    ))
////                    drive.poseEstimate = Pose2d(poseEstimate.x, poseEstimate.y, poseEstimate.heading)
//                }),
////                ScheduleCommand(FollowTrajectorySequence(drive,
////                    drive.trajectorySequenceBuilder(drive.poseEstimate)
////                        .setReversed(true)
////                        .lineTo(Vector2d(poseEstimate.x, poseEstimate.y))
////                        .setReversed(false)
////                        .splineTo(Vector2d(0.0, 34.5), Math.toRadians(180.0))
////                        .splineTo(Vector2d(-59.0, 34.0), Math.toRadians(180.0))
////                        .setReversed(true)
////                        .splineTo(Vector2d(0.0, 36.0), Math.toRadians(0.0))
////                        .splineTo(Vector2d(48.0, 36.0), Math.toRadians(0.0))
////                        .build()
////                )),
////                WaitCommand(500),
////                FollowTrajectorySequence(drive, moveToTag),
//                WaitCommand(1000),
//            )
//        )
//    }
//}