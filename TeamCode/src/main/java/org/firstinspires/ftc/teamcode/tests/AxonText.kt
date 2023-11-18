package org.firstinspires.ftc.teamcode.tests

import android.graphics.Color
import android.util.Log
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@Config
@Autonomous
class AxonText : LinearOpMode() {
    @OptIn(ExperimentalStdlibApi::class)
    override fun runOpMode() {
        var servo = hardwareMap.get(Servo::class.java,"servo")
        var analog = hardwareMap.get(AnalogInput::class.java, "analog")
        var timer = ElapsedTime()
        waitForStart()
        while (opModeIsActive()){

        }
    }
}