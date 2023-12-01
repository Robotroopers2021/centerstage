package org.firstinspires.ftc.teamcode.tests

import android.util.Log
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.subsystems.cv.AprilTag
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeDetector
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeProcessor
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive

@Config
@Autonomous
class SpikeTest : LinearOpMode() {
    override fun runOpMode() {
        val cv = SpikeDetector(hardwareMap, SpikeProcessor.Color.RED)
        waitForStart()
        while (opModeIsActive()){
            telemetry.addData("Position", cv.position.toString())
        }
    }

}