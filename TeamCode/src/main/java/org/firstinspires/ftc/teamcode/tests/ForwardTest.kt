package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.alphago.agDistanceLocalization.filters.MedianFilter
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DistanceSensor
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.subsystems.cv.AprilTag
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive
import kotlin.math.min

@TeleOp
class ForwardTest : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        val cv = AprilTag(hardwareMap, this)
        var channel = cv.channel
        var yDist: Double?
        var detection: Map<Int, Vector2d>
        var dist = hardwareMap.get(DistanceSensor::class.java, "dist") as MB1242Ex
        //var lowPassFilter = LowPassFilter(0.75)
        var medianFilter = MedianFilter(10)
        waitForStart()
        while (opModeIsActive()) {
            detection = runBlocking {
                channel.receive()
            }
            yDist = detection.values.minOfOrNull { it.y }
            if (yDist != null) {
                if(gamepad1.circle || yDist > 10) {
                    drive.setDrivePowers(
                        PoseVelocity2d(
                            Vector2d(
                                -gamepad1.left_stick_y.toDouble(),
                                -gamepad1.left_stick_x.toDouble()
                            ),
                            -gamepad1.right_stick_x.toDouble()
                        )
                    )
                } else{
                    drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
                }
            }
            drive.updatePoseEstimate()
            telemetry.addData("x", drive.pose.position.x)
            telemetry.addData("y", drive.pose.position.y)
            telemetry.addData("heading", drive.pose.heading)
            telemetry.addData("fps", cv.visionPortal.fps)
            telemetry.addData("yDist", yDist)
            telemetry.update()
        }
    }
}