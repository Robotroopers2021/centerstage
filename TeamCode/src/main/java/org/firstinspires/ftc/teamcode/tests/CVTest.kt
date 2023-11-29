package org.firstinspires.ftc.teamcode.tests

import android.util.Log
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.subsystems.cv.AprilTag
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive

@Config
@Autonomous
class CVTest : LinearOpMode() {
    override fun runOpMode() {
        val cv = AprilTag(hardwareMap, this, telemetry, AprilTag.Mode.ASYNC)
        val drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        waitForStart()
        while (opModeIsActive()){
            drive.setDrivePowers(
                PoseVelocity2d(
                    Vector2d(
                        -gamepad1.left_stick_y.toDouble(),
                        -gamepad1.left_stick_x.toDouble()
                    ),
                    -gamepad1.right_stick_x.toDouble()
                )
            )
            Log.d("cvPose", cv.getPose(5).toString())
        }
    }

}