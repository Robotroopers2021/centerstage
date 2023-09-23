package org.firstinspires.ftc.teamcode.subsystems.cv

import android.util.Log
import android.util.Size
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

@OptIn(DelicateCoroutinesApi::class)
class AprilTag(hardwareMap: HardwareMap) {
    var aprilTag: AprilTagProcessor = AprilTagProcessor.Builder()
        .setDrawAxes(true)
        .setDrawCubeProjection(true)
        .setDrawTagOutline(true)
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
        .build()
    var visionPortal: VisionPortal

    init {
        val builder = VisionPortal.Builder()

        builder.setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
        builder.setCameraResolution(Size(1920, 1080))
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG)

        builder.addProcessor(aprilTag)

        visionPortal = builder.build()
    }

    /**
     * Return pose of april tag relative robot (local camera) pos
     * not the global position
     */
    fun detect(): Map<Int, Twist2d> {
        var map = mutableMapOf<Int, Twist2d>()
        GlobalScope.launch(Dispatchers.Default){
            Log.i("AprilTag", Thread.currentThread().name.toString())

            aprilTag.detections.forEach {
                map[it.id] = Twist2d(Vector2d( it.ftcPose.x, it.ftcPose.y), it.ftcPose.yaw)
            }
        }
        return map
    }
}