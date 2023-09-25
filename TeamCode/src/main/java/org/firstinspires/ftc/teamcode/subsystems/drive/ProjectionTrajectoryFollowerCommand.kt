package org.firstinspires.ftc.teamcode.subsystems.drive

import com.acmerobotics.roadrunner.DisplacementTrajectory
import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Trajectory
import com.acmerobotics.roadrunner.Twist2dDual
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.CommandBase
import kotlinx.coroutines.channels.Channel


class ProjectionTrajectoryFollowerCommand(var drive: MecanumDriveSubsystem, t: Trajectory, private val channel: Channel<Vector2d>) :
    CommandBase() {
    var dt = DisplacementTrajectory(t)
    var c = HolonomicController(
        MecanumDrive.PARAMS.axialGain,
        MecanumDrive.PARAMS.lateralGain,
        MecanumDrive.PARAMS.headingGain,
        MecanumDrive.PARAMS.axialVelGain,
        MecanumDrive.PARAMS.lateralVelGain,
        MecanumDrive.PARAMS.headingVelGain)
    var disp = 0.0

    init {
        addRequirements(drive)
    }

    override fun initialize() {
    }

    override fun execute() {
        /*val robotVelRobot = updatePoseEstimateAndGetActualVel()
        var pose = {channel.receive()}.invoke()
        disp = dt.project(, disp)
        val poseTarget = dt[disp]
        val cmd = c.compute(poseTarget, pose, robotVelRobot)

        val wheelVels: WheelVelocities<Time> = kinematics.inverse(cmd)
        val voltage: Double = voltageSensor.getVoltage()
        drive.drive.leftFront.power = drive.drive.feedforward.compute(wheelVels.leftFront) / voltage
        drive.drive.leftBack.power = drive.drive.feedforward.compute(wheelVels.leftBack) / voltage
        drive.drive.rightBack.power = drive.drive.feedforward.compute(wheelVels.rightBack) / voltage
        drive.drive.rightFront.power = drive.drive.feedforward.compute(wheelVels.rightFront) / voltage*/
    }

    override fun isFinished(): Boolean {
        return disp + 1 < dt.length()
    }

    override fun end(interrupted: Boolean) {
        drive.stop()
    }
}