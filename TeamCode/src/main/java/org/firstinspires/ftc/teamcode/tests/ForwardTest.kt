package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.alphago.agDistanceLocalization.filters.LowPassFilter
import com.alphago.agDistanceLocalization.filters.MedianFilter
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive

@TeleOp
class ForwardTest : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        //val cv = AprilTag(hardwareMap)
        var yDist: Double?
        var detection: MutableMap<Int, Twist2d>
        var dist = hardwareMap.get(DistanceSensor::class.java, "dist") as MB1242Ex
        //var lowPassFilter = LowPassFilter(0.75)
        var medianFilter = MedianFilter(10)
        waitForStart()
        while (opModeIsActive()) {
            //detection = cv.map
            //yDist = detection[2]?.line?.y
            //medianFilter.push(dist.getDistanceAsync(DistanceUnit.INCH))
            yDist = medianFilter.median() //lowPassFilter.estimate(dist.getDistanceAsync(DistanceUnit.INCH))
            if(gamepad1.circle || yDist > 5) {
                drive.setDrivePowers(
                    PoseVelocity2d(
                        Vector2d(
                            -gamepad1.left_stick_y.toDouble(),
                            -gamepad1.left_stick_x.toDouble()
                        ),
                        -gamepad1.right_stick_x.toDouble()
                    )
                )
            }
            else{
                drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
            }
            drive.updatePoseEstimate()
            telemetry.addData("x", drive.pose.position.x)
            telemetry.addData("y", drive.pose.position.y)
            telemetry.addData("heading", drive.pose.heading)
            telemetry.addData("yDist", yDist)
            telemetry.update()
        }
    }
}