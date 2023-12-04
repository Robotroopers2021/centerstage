package org.firstinspires.ftc.teamcode.tests

import android.util.Log
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.subsystems.cv.Spike
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeProcessor

@Config
@Autonomous
class SpikeTest : LinearOpMode() {
    override fun runOpMode() {
        val cv = Spike(hardwareMap, SpikeProcessor.Color.RED)
        waitForStart()
        while (opModeIsActive()){
            telemetry.addData("Position", cv.position.toString())
            Log.d("SpikeTest", cv.position.toString())
            telemetry.update()
        }
    }

}