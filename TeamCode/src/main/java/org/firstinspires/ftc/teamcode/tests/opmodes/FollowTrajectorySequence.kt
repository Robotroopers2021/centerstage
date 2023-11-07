//package org.firstinspires.ftc.teamcode.tests.opmodes
//
//import com.arcrobotics.ftclib.command.CommandBase
//import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence
//
//
//class FollowTrajectorySequence(drive: MecanumDrive, trajectorySequence: TrajectorySequence): CommandBase() {
//    private var drive: MecanumDrive
//    private var trajectorySequence : TrajectorySequence
//    init {
//        this.drive = drive
//        this.trajectorySequence = trajectorySequence
//        addRequirements(drive)
//    }
//
//    override fun initialize() {
//        drive.followTrajectorySequenceAsync(trajectorySequence)
//    }
//
//    override fun execute() {
//        drive.update()
//    }
//
//    override fun end(interrupted: Boolean) {
//        if (interrupted) {
//            drive.stop()
//        }
//    }
//
//    override fun isFinished(): Boolean {
//        return Thread.currentThread().isInterrupted || !drive.isBusy
//    }
//}