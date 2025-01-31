package org.firstinspires.ftc.teamcode.subsystems.cv

import android.util.Log
import android.util.Size
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.Telemetry
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
class AprilTag(hardwareMap: HardwareMap, opMode: LinearOpMode, val telemetry: Telemetry, var mode: Mode = Mode.NORMAL) {
    enum class Mode {
        NORMAL,
        ASYNC
    }

    enum class Camera {
        FRONT,
        BACK
    }

    companion object {
        val FRONT_CAM_OFFSET = Twist2d(Vector2d(0.0, 0.0), 0.0)
        val BACK_CAM_OFFSET = Twist2d(Vector2d(0.0, 0.0), 0.0)
    }

    var webcamFront: WebcamName
    var webcamBack: WebcamName


    var aprilTag: AprilTagProcessor = AprilTagProcessor.Builder()
        .setDrawAxes(false)
        .setDrawCubeProjection(false)
        .setDrawTagOutline(false)
        .setDrawTagID(false)
        //.setLensIntrinsics(723.5042, 725.4909, 404.5462, 313.1578) //800x600
        .setLensIntrinsics(292.4088, 292.7053, 159.1876, 124.7638) //320x240
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
        .setNumThreads(6)
        .build()

    var currentCamera = Camera.FRONT
        set(value) = when(value){
                Camera.FRONT -> visionPortal.activeCamera = webcamFront
                Camera.BACK -> visionPortal.activeCamera = webcamBack
            }



    @Volatile
    var visionPortal: VisionPortal


    @Volatile
    var channel = Channel<Map<Int, Pose2d>>() //Used for async mode

    init {
        val builder = VisionPortal.Builder()

        webcamFront = hardwareMap.get(WebcamName::class.java, "Webcam 1")
        webcamBack = hardwareMap.get(WebcamName::class.java, "Webcam 1")
        val switchableCamera: CameraName = ClassFactory.getInstance()
            .cameraManager.nameForSwitchableCamera(webcamFront, webcamBack)

        builder.setCamera(switchableCamera)

        builder.setCameraResolution(Size(320, 240))
        //builder.setCameraResolution(Size(800, 600))
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        //aprilTag.setDecimation(10F)
        builder.addProcessor(aprilTag)

        builder.enableLiveView(false)
        visionPortal = builder.build()
        visionPortal.setProcessorEnabled(aprilTag, true)

        while (visionPortal.cameraState != VisionPortal.CameraState.STREAMING) {
        }
        visionPortal.getCameraControl(ExposureControl::class.java).mode =
            ExposureControl.Mode.Manual
        sleep(50)
        visionPortal.getCameraControl(ExposureControl::class.java).setExposure(
            visionPortal.getCameraControl(ExposureControl::class.java)
                .getMinExposure(TimeUnit.MILLISECONDS), TimeUnit.MILLISECONDS
        )

        //TODO: Make Async work
        if (mode == Mode.ASYNC)
            GlobalScope.launch(Dispatchers.Default) {
                Log.d("APRILTAG", "CV THREAD STARTED")
                Log.d("APRILTAG", opMode.isStarted.toString())
                while (opMode.opModeIsActive()) {
                    channel.send(buildMap {
                        aprilTag.detections.forEach {
                            put(it.id, Pose2d(it.ftcPose.x, -it.ftcPose.y, it.ftcPose.yaw))
                        }
                    })
                    Log.d("APRILTAG", aprilTag.detections.size.toString())
                }
            }
    }

    /*fun setCamera(camera: Camera) {
        when (camera) {
            Camera.FRONT -> {
                visionPortal.activeCamera = webcamFront
                currentCamera = Camera.FRONT
            }

            Camera.BACK -> {
                visionPortal.activeCamera = webcamBack
                currentCamera = Camera.BACK
            }
        }
    }*/

    //TODO: Make the pose offset calculation work
    //Gets pose offset from tag to center of robot
    fun getPose(id: Int): Pose2d? {
        //Log.d("cvPose", "CALLED GET POSE")
        val detections = aprilTag.detections
        when (mode) {
            Mode.NORMAL -> {
                val detectMap = buildMap {
                    detections.forEach {
                        put(it.id, Pose2d(it.ftcPose.y, -it.ftcPose.x, it.ftcPose.yaw))
                        Log.d("cvPose${it.id}", Pose2d(it.ftcPose.y, -it.ftcPose.x, it.ftcPose.yaw).toString())
                        if (it.metadata != null) {
                            telemetry.addLine(
                                String.format(
                                    "\n==== (ID %d) %s",
                                    it.id,
                                    it.metadata.name
                                )
                            )
                            telemetry.addLine(
                                String.format(
                                    "XYZ %6.1f %6.1f %6.1f  (inch)",
                                    it.ftcPose.x,
                                    it.ftcPose.y,
                                    it.ftcPose.z
                                )
                            )
                            telemetry.addLine(
                                String.format(
                                    "PRY %6.1f %6.1f %6.1f  (deg)",
                                    it.ftcPose.pitch,
                                    it.ftcPose.roll,
                                    it.ftcPose.yaw
                                )
                            )
                            telemetry.addLine(
                                String.format(
                                    "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                                    it.ftcPose.range,
                                    it.ftcPose.bearing,
                                    it.ftcPose.elevation
                                )
                            )
                        } else {
                            telemetry.addLine(String.format("\n==== (ID %d) Unknown", it.id))
                            telemetry.addLine(
                                String.format(
                                    "Center %6.0f %6.0f   (pixels)",
                                    it.center.x,
                                    it.center.y
                                )
                            )
                        }
                    }
                }
                return when (currentCamera) {
                    Camera.FRONT -> detectMap[id]?.plus(FRONT_CAM_OFFSET)
                    Camera.BACK -> detectMap[id]?.plus(BACK_CAM_OFFSET)
                }
            }

            Mode.ASYNC -> {
                return runBlocking {
                    when (currentCamera) {
                        Camera.FRONT -> channel.receive()[id]?.plus(FRONT_CAM_OFFSET)
                        Camera.BACK -> channel.receive()[id]?.plus(BACK_CAM_OFFSET)
                    }
                }
            }
        }
    }
}