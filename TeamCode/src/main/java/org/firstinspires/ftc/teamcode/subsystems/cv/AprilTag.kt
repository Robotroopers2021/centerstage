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
        /*.setDrawAxes(true)
        .setDrawCubeProjection(true)
        .setDrawTagOutline(true)*/
        .setNumThreads(6)
        .setLensIntrinsics(876.376640148741, 879.046105394020, 633.420091455433, 408.814246529796)
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
        .build()

    var visionPortal: VisionPortal
    @Volatile var channel = Channel<Map<Int, Vector2d>>()

    init {
        val builder = VisionPortal.Builder()

        builder.setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))


        builder.setCameraResolution(Size(320, 240))
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG)

        builder.addProcessor(aprilTag)

        visionPortal = builder.build()

        /*var session = Continuation.create({}, object: StateCallback{
            override fun onConfigured(session: CameraCaptureSession) {
            }
            override fun onClosed(session: CameraCaptureSession) {
            }
        })

        val captureSession = cam.createCaptureSession(session)

        cam.createCaptureRequest(JPEG, org.firstinspires.ftc.robotcore.external.android.util.Size(1200, 800), 120)*/


        GlobalScope.launch(Dispatchers.Default){
            while (opMode.opModeIsActive()) {
                channel.send(buildMap {
                    aprilTag.detections.forEach {
                        put(it.id, Vector2d(it.ftcPose.x, it.ftcPose.y))
                    }
                })

            }
        }
    }
}