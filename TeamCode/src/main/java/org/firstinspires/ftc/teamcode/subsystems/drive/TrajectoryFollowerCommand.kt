package org.firstinspires.ftc.teamcode.subsystems.drive

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive.FollowTrajectoryAction

class TrajectoryFollowerCommand(private val drive: MecanumDriveSubsystem, action: FollowTrajectoryAction) :
    CommandBase() {

    init {
        addRequirements(drive)
    }

    override fun initialize() {
        drive.followTrajectory(trajectory)
    }

    override fun execute() {
        drive.update()
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            drive.stop()
        }
    }

    override fun isFinished(): Boolean {
        return Thread.currentThread().isInterrupted || !drive.
    }
}