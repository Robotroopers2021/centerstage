package org.firstinspires.ftc.teamcode.subsystems.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.DisplacementTrajectory
import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.Trajectory
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.FlightRecorder.write
import com.arcrobotics.ftclib.command.CommandBase
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking


class ProjectionTrajectoryFollowerCommand(var drive: MecanumDriveSubsystem, t: Trajectory, private val channel: Channel<Vector2d>) :
    CommandBase() {
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

    init {
        addRequirements(drive)
    }

    override fun execute() {
        val p = TelemetryPacket()
        p.fieldOverlay().operations.addAll(c.operations)

        val robotVelRobot = drive.updatePoseEstimate()
        runBlocking{launch {cvPose = channel.receive()}}
        disp = dt.project(cvPose, disp)
        val poseTarget = dt[disp]
        var pose = drive.poseEstimate
        val cmd = hc.compute(poseTarget, pose, robotVelRobot)

        val wheelVels = drive.drive.kinematics.inverse(cmd)
        val voltage = drive.drive.voltageSensor.voltage
        drive.drive.leftFront.power = drive.drive.feedforward.compute(wheelVels.leftFront) / voltage
        drive.drive.leftBack.power = drive.drive.feedforward.compute(wheelVels.leftBack) / voltage
        drive.drive.rightBack.power = drive.drive.feedforward.compute(wheelVels.rightBack) / voltage
        drive.drive.rightFront.power = drive.drive.feedforward.compute(wheelVels.rightFront) / voltage
        write("TARGET_POSE", PoseMessage(poseTarget.value()))

        p.put("x", pose.position.x)
        p.put("y", pose.position.y)
        p.put("heading (deg)", Math.toDegrees(pose.heading.log()))

        val (position, heading) = poseTarget.value().minusExp(pose)
        p.put("xError", position.x)
        p.put("yError", position.y)
        p.put("headingError (deg)", Math.toDegrees(heading.log()))

        // only draw when active; only one drive action should be active at a time

        // only draw when active; only one drive action should be active at a time
        val c = p.fieldOverlay()
        drive.drive.drawPoseHistory(c)

        c.setStroke("#4CAF50")
        MecanumDrive.drawRobot(c, poseTarget.value())

        c.setStroke("#3F51B5")
        MecanumDrive.drawRobot(c, pose)

        dash.sendTelemetryPacket(p)
    }

    override fun isFinished(): Boolean {
        return disp + 1 >= dt.length()
    }

    override fun end(interrupted: Boolean) {
        drive.stop()
    }
}