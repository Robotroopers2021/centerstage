package org.firstinspires.ftc.teamcode.subsystems.cv

import android.util.Log
import android.util.Size
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.launch
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.lang.Thread.sleep
import java.util.Locale.Builder
import java.util.concurrent.TimeUnit

@OptIn(DelicateCoroutinesApi::class)
class AprilTag(hardwareMap: HardwareMap, opMode: LinearOpMode) {
    var aprilTag: AprilTagProcessor = AprilTagProcessor.Builder()
        .setDrawAxes(false)
        .setDrawCubeProjection(false)
        .setDrawTagOutline(false)
        .setDrawTagID(false)
        .setLensIntrinsics(723.5042, 725.4909, 404.5462, 313.1578) //800x600
        //.setLensIntrinsics(292.4088, 292.7053, 159.1876, 124.7638) //320x240
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
        .setNumThreads(6)
        .build()


    @Volatile var visionPortal: VisionPortal
    @Volatile var channel = Channel<Map<Int, Pose2d>>()

    init {
        val builder = VisionPortal.Builder()

        builder.setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))

        //builder.setCameraResolution(Size(320, 240))
        builder.setCameraResolution(Size(800, 600))
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        //aprilTag.setDecimation(10F)
        builder.addProcessor(aprilTag)

        visionPortal = builder.build()
        while(visionPortal.cameraState != VisionPortal.CameraState.STREAMING){}
        visionPortal.getCameraControl(ExposureControl::class.java).mode = ExposureControl.Mode.Manual
        sleep(50)
        visionPortal.getCameraControl(ExposureControl::class.java).setExposure(visionPortal.getCameraControl(ExposureControl::class.java).getMinExposure(TimeUnit.MILLISECONDS), TimeUnit.MILLISECONDS)



        /*GlobalScope.launch(Dispatchers.Default){
            Log.d("AUTOTEST", "CV THREAD STARTED")
            Log.d("AUTOTEST", opMode.isStarted.toString())
            while (true) {
                channel.send(buildMap {
                    aprilTag.detections.forEach {
                        put(it.id, Pose2d(it.ftcPose.x, -it.ftcPose.y, it.ftcPose.yaw))
                    }
                })
                Log.d("AUTOTEST", aprilTag.detections.size.toString())
                if (aprilTag.detections.size>0)
                    aprilTag.detections.forEach {
                        Log.d("AUTOTEST", Pose2d(it.ftcPose.x, -it.ftcPose.y, it.ftcPose.yaw).toString())
                    }
            }
        }*/
    }
}