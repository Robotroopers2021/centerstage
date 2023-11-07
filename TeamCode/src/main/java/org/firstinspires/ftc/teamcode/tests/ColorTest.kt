package org.firstinspires.ftc.teamcode.tests

import android.graphics.Color
import android.util.Log
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@Config
@Autonomous
class ColorTest : LinearOpMode() {
    @OptIn(ExperimentalStdlibApi::class)
    override fun runOpMode() {
        var colorLeft = hardwareMap.get(NormalizedColorSensor::class.java, "colorLeft")

        waitForStart()
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
        val hsvValues = FloatArray(3)
        while (opModeIsActive()){
            Color.colorToHSV(colorLeft.normalizedColors.toColor(), hsvValues)
            var hue = hsvValues[0]
            Log.d("ColorMonkey", colorLeft.normalizedColors.toColor().toHexString(HexFormat.UpperCase).substring(2))
            if (hue>200)
                telemetry.addData("color", "<font color=#800080>purple</font>")
            else if(hue>135)
                telemetry.addData("color", "<font color=#FFFFFF>white</font>")
            else if(hue>100)
                telemetry.addData("color", "<font color=#008000>green</font>")
            else
                telemetry.addData("color", "<font color=#FFFF00>yellow</font>")
            telemetry.addData("Dist ", (colorLeft as DistanceSensor).getDistance(DistanceUnit.INCH))
            telemetry.update()
        }
    }
}