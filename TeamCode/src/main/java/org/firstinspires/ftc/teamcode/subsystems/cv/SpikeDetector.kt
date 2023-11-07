package org.firstinspires.ftc.teamcode.subsystems.cv

import android.util.Log
import android.util.Size
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.alphago.agDistanceLocalization.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.lang.Thread.sleep
import java.util.concurrent.TimeUnit
import kotlin.math.PI

@OptIn(DelicateCoroutinesApi::class)
class SpikeDetector(hardwareMap: HardwareMap, color: SpikeProcessor.Color) {
    var visionPortal: VisionPortal
    var processor = SpikeProcessor(color)
    init {
        val builder = VisionPortal.Builder()

        var webcam = hardwareMap.get(WebcamName::class.java, "Webcam 3")
        builder.setCamera(webcam)

        builder.setCameraResolution(Size(1920, 1080))
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        builder.addProcessor(SpikeProcessor(color))

        visionPortal = builder.build()
    }

    fun stop(){
        visionPortal.stopStreaming()
    }
    val position: SpikeProcessor.Position
        get() = processor.position
}