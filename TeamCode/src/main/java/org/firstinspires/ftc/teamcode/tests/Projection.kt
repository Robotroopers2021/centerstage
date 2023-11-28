package org.firstinspires.ftc.teamcode.tests

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.DisplacementTrajectory
import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.subsystems.cv.AprilTag
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.drive.PoseMessage
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import kotlin.math.PI

@Config
@Autonomous
class Projection : LinearOpMode() {
    override fun runOpMode() {
        val targetID = 5
        val target = 5
        val cv = AprilTag(hardwareMap, this, telemetry)
        Log.d("cvPose", "init cv")
        while(cv.getPose(target) == null){}
        val startPos = Pose2d(cv.getPose(target)!!.position+Vector2d(
            AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(target).fieldPosition.get(0).toDouble(),
            AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(target).fieldPosition.get(1).toDouble(),
        ), 0.0)
        Log.d("cvPose", startPos.toString())
        val drive = MecanumDrive(hardwareMap, startPos)
        val t = TrajectoryBuilder(
            startPos, 1e-6, 0.0,
            drive.defaultVelConstraint, drive.defaultAccelConstraint, 0.25, 0.1
        ).splineTo(Vector2d(
            AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(target).fieldPosition.get(0).toDouble()+5,
            AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(target).fieldPosition.get(1).toDouble(),
        ), PI).build()[0]
        var dt = DisplacementTrajectory(t)
        var hc = HolonomicController(
            MecanumDrive.PARAMS.axialGain,
            MecanumDrive.PARAMS.lateralGain,
            MecanumDrive.PARAMS.headingGain,
            MecanumDrive.PARAMS.axialVelGain,
            MecanumDrive.PARAMS.lateralVelGain,
            MecanumDrive.PARAMS.headingVelGain)
        var disp = 0.0

        val dash = FtcDashboard.getInstance()
        val c = Canvas()
        var b = true
        lateinit var cvPose: Vector2d
        waitForStart()
        while (disp + 1 < dt.length()){
            val p = TelemetryPacket()
            p.fieldOverlay().operations.addAll(c.operations)

            val robotVelRobot = drive.updatePoseEstimate()
            //cvPose = Vector2d(aprilTag.aprilTag.detections[0].ftcPose.x, -aprilTag.aprilTag.detections[0].ftcPose.y)

            Log.d("cvPose", "Getting pose")
            cvPose = cv.getPose(5)!!.position
            Log.d("cvPose", cvPose.toString())
            cvPose += Vector2d(
                AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(targetID).fieldPosition.get(0)
                    .toDouble(),
                AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(targetID).fieldPosition.get(1)
                    .toDouble(),
            )

            disp = dt.project(cvPose, disp)
            val poseTarget = dt[disp]
            val pose = drive.pose
            val cmd = hc.compute(poseTarget, pose, robotVelRobot)

            val wheelVels = drive.kinematics.inverse(cmd)
            val voltage = drive.voltageSensor.voltage
            val feedforward = MotorFeedforward(
                MecanumDrive.PARAMS.kS,
                MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick
            )
            drive.leftFront.power = feedforward.compute(wheelVels.leftFront) / voltage
            drive.leftBack.power = feedforward.compute(wheelVels.leftBack) / voltage
            drive.rightBack.power = feedforward.compute(wheelVels.rightBack) / voltage
            drive.rightFront.power = feedforward.compute(wheelVels.rightFront) / voltage
            FlightRecorder.write("TARGET_POSE", PoseMessage(poseTarget.value()))

            p.put("x", pose.position.x)
            p.put("y", pose.position.y)
            p.put("heading (deg)", Math.toDegrees(pose.heading.log()))

            val (position, heading) = poseTarget.value().minusExp(pose)
            p.put("xError", position.x)
            p.put("yError", position.y)
            p.put("headingError (deg)", Math.toDegrees(heading.log()))


            dash.sendTelemetryPacket(p)
        }
        Log.d("cvPose", "Done")
        drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
        cv.visionPortal.close()
    }

}