package org.firstinspires.ftc.teamcode.subsystems.cv

import android.util.Log
import android.util.Size
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.launch
import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

@OptIn(DelicateCoroutinesApi::class)
class AprilTag(hardwareMap: HardwareMap, opMode: LinearOpMode) {
    var aprilTag: AprilTagProcessor = AprilTagProcessor.Builder()
        .setDrawAxes(false)
        .setDrawCubeProjection(false)
        .setDrawTagOutline(false)
        .setDrawTagID(false)
        //.setLensIntrinsics(723.5042, 725.4909, 404.5462, 313.1578) 800x600
        .setLensIntrinsics(292.4088, 292.7053, 159.1876, 124.7638)
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
        .build()

    var visionPortal: VisionPortal
    @Volatile var channel = Channel<Map<Int, Vector2d>>()

    init {
        val builder = VisionPortal.Builder()

        builder.setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))


        //builder.setCameraResolution(Size(320, 240))
        builder.setCameraResolution(Size(800, 600))
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG)

        builder.addProcessor(aprilTag)

        visionPortal = builder.build()


        GlobalScope.launch(Dispatchers.Default){
            while (true) {
                channel.send(buildMap {
                    aprilTag.detections.forEach {
                        put(it.id, Vector2d(it.ftcPose.x, it.ftcPose.y))
                    }
                })

            }
        }
    }
}