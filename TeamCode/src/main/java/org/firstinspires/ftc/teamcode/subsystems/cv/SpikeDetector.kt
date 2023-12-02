package org.firstinspires.ftc.teamcode.subsystems.cv

import android.util.Size
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.DelicateCoroutinesApi
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal

@OptIn(DelicateCoroutinesApi::class)
class SpikeDetector(hardwareMap: HardwareMap, color: SpikeProcessor.Color) {
    var visionPortal: VisionPortal
    var processor = SpikeProcessor(color)
    init {
        val builder = VisionPortal.Builder()

        var webcam = hardwareMap.get(WebcamName::class.java, "Webcam 3")
        builder.setCamera(webcam)

        builder.setCameraResolution(Size(1920, 1080))
        //builder.setCameraResolution(Size(1280, 960))
        builder.addProcessor(SpikeProcessor(color))

        visionPortal = builder.build()
    }

    fun stop(){
        visionPortal.stopStreaming()
    }
    val position: SpikeProcessor.Position
        get() = processor.position
}