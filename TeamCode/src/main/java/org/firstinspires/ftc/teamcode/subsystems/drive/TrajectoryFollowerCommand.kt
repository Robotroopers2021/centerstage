package org.firstinspires.ftc.teamcode.subsystems.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive.FollowTrajectoryAction

class TrajectoryFollowerCommand(private val drive: MecanumDriveSubsystem, private val trajectory: FollowTrajectoryAction) :
    CommandBase() {
    val dash = FtcDashboard.getInstance()
    val c = Canvas()
    var b = true
    init {
        addRequirements(drive)
    }

    override fun initialize() {
        trajectory.preview(c)
    }

    override fun execute() {
        val p = TelemetryPacket()
        p.fieldOverlay().operations.addAll(c.operations)

        b = trajectory.run(p)

        dash.sendTelemetryPacket(p)
    }

    override fun isFinished(): Boolean {
        return !b || Thread.currentThread().isInterrupted
    }
}