package org.firstinspires.ftc.teamcode.tests.opmodes

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
import com.acmerobotics.roadrunner.ftc.*
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.DepositCmd
import org.firstinspires.ftc.teamcode.commands.HomeCmd
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm
import org.firstinspires.ftc.teamcode.subsystems.cv.AprilTag
import org.firstinspires.ftc.teamcode.subsystems.cv.Spike
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeProcessor
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.PoseMessage
import org.firstinspires.ftc.teamcode.subsystems.drive.commands.TrajectoryFollowerCommand
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift
import org.firstinspires.ftc.teamcode.subsystems.wrist.Wrist
import org.firstinspires.ftc.teamcode.tests.opmodes.BlueConstants.depositX
import org.firstinspires.ftc.teamcode.tests.opmodes.BlueConstants.depositCenterY
import org.firstinspires.ftc.teamcode.tests.opmodes.BlueConstants.depositLeftY
import org.firstinspires.ftc.teamcode.tests.opmodes.BlueConstants.depositRightY
import org.firstinspires.ftc.teamcode.tests.opmodes.BlueConstants.parkX
import org.firstinspires.ftc.teamcode.tests.opmodes.BlueConstants.parkY
import org.firstinspires.ftc.teamcode.tests.opmodes.BlueConstants.spikeLeftX
import org.firstinspires.ftc.teamcode.tests.opmodes.BlueConstants.spikeLeftY
import org.firstinspires.ftc.teamcode.tests.opmodes.BlueConstants.spikeMiddleX
import org.firstinspires.ftc.teamcode.tests.opmodes.BlueConstants.spikeMiddleY
import org.firstinspires.ftc.teamcode.tests.opmodes.BlueConstants.spikeRightX
import org.firstinspires.ftc.teamcode.tests.opmodes.BlueConstants.spikeRightY
import org.firstinspires.ftc.teamcode.tests.opmodes.BlueConstants.startH
import org.firstinspires.ftc.teamcode.tests.opmodes.BlueConstants.startX
import org.firstinspires.ftc.teamcode.tests.opmodes.BlueConstants.startY
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin


@Config
@Autonomous
class AutoTest : CommandOpMode() {
    private lateinit var drive: MecanumDriveSubsystem
    private lateinit var spike: Spike
    private lateinit var lift: Lift
    private lateinit var arm: Arm
    private lateinit var wrist: Wrist
    private lateinit var intake: Intake
    private lateinit var aprilTag: AprilTag


    override fun initialize() {
        val startPos = Pose2d(startX, startY, startH)
        arm = Arm(hardwareMap, telemetry)
        wrist = Wrist(hardwareMap, telemetry)
        lift = Lift(hardwareMap, telemetry, Lift.opModeType.Autonomous)
        spike = Spike(hardwareMap, SpikeProcessor.Color.RED)
        intake = Intake(hardwareMap, telemetry)
        aprilTag = AprilTag(hardwareMap, this, telemetry)
        aprilTag.visionPortal.stopStreaming()
        drive = MecanumDriveSubsystem(hardwareMap, startPos, false)
        drive.poseEstimate = startPos

        val spikeLeft = drive.drive.actionBuilder(drive.poseEstimate)
            .setReversed(true)
            .strafeToLinearHeading(Vector2d(spikeLeftX, spikeLeftY), Math.toRadians(180.0))
            /*.waitSeconds(1.0)
            .strafeToLinearHeading(Vector2d(47.0, 43.0), Math.toRadians(180.0))
            //CYCLE 1
            .setReversed(false)
            .splineToConstantHeading(Vector2d(24.0, 6.5), Math.toRadians(180.0))
            .strafeToConstantHeading(Vector2d(-56.0, 6.5))
            .waitSeconds(1.0)
            .setReversed(true)
            .strafeToConstantHeading(Vector2d(24.0, 6.5))
            .splineToConstantHeading(Vector2d(48.0, 36.5), Math.toRadians(90.0))
            //CYCLE 2
            .setReversed(false)
            .splineToConstantHeading(Vector2d(24.0, 6.5), Math.toRadians(180.0))
            .strafeToConstantHeading(Vector2d(-56.0, 6.5))
            .waitSeconds(1.0)
            .setReversed(true)
            .strafeToConstantHeading(Vector2d(24.0, 6.5))
            .splineToConstantHeading(Vector2d(48.0, 36.5), Math.toRadians(90.0))*/
            .build()

        val spikeMiddle = drive.drive.actionBuilder(drive.poseEstimate)
            .setReversed(true)
            .strafeToLinearHeading(Vector2d(spikeMiddleX, spikeMiddleY), Math.toRadians(90.0))
            .build()

        val spikeRight = drive.drive.actionBuilder(drive.poseEstimate)
            .setReversed(true)
            .strafeToLinearHeading(Vector2d(spikeRightX, spikeRightY), Math.toRadians(0.0))
            .build()

        while (opModeInInit()){
            telemetry.addData("Spike Position", spike.position.toString())
            telemetry.update()
        }

        waitForStart()
        spike.stop()
        aprilTag.visionPortal.resumeStreaming()
        aprilTag.currentCamera = AprilTag.Camera.BACK

        val deposit = drive.drive.actionBuilder(when(spike.position){
            SpikeProcessor.Position.LEFT -> Pose2d(spikeLeftX, spikeLeftY, Math.toRadians(180.0))
            SpikeProcessor.Position.CENTER -> Pose2d(spikeMiddleX, spikeMiddleY, Math.toRadians(90.0))
            SpikeProcessor.Position.RIGHT -> Pose2d(spikeRightX, spikeRightY, Math.toRadians(0.0))
        })
            .setReversed(true)
            .strafeTo(when(spike.position){
                SpikeProcessor.Position.LEFT -> Vector2d(spikeLeftX-5, spikeLeftY+10)
                SpikeProcessor.Position.CENTER -> Vector2d(spikeMiddleX, spikeMiddleY+5)
                SpikeProcessor.Position.RIGHT -> Vector2d(spikeRightX+5, spikeRightY+10)
            })
            //.splineTo(Vector2d(depositX, depositY), Math.toRadians(180.0))
            /*.splineToLinearHeading(Pose2d(when(spike.position){
                SpikeProcessor.Position.LEFT -> Vector2d(spikeLeftX-5, spikeLeftY)
                SpikeProcessor.Position.CENTER -> Vector2d(depositCenterX, depositCenterY)
                SpikeProcessor.Position.RIGHT -> Vector2d(spikeRightX+5, spikeRightY)
            }, Math.toRadians(180.0)), Math.toRadians(180.0))*/

            .strafeToLinearHeading(when(spike.position){
                SpikeProcessor.Position.LEFT -> Vector2d(depositX, depositLeftY)
                SpikeProcessor.Position.CENTER -> Vector2d(depositX, depositCenterY)
                SpikeProcessor.Position.RIGHT -> Vector2d(depositX, depositRightY)
            }, Math.toRadians(180.0))
            .build()

        val park = drive.drive.actionBuilder(when(spike.position){
            SpikeProcessor.Position.LEFT -> Pose2d(spikeLeftX, spikeLeftY, Math.toRadians(180.0))
            SpikeProcessor.Position.CENTER -> Pose2d(spikeMiddleX, spikeMiddleY, Math.toRadians(180.0))
            SpikeProcessor.Position.RIGHT -> Pose2d(spikeRightX, spikeRightY, Math.toRadians(180.0))
        })
            .setReversed(true)
            .strafeTo(Vector2d(parkX, parkY))
            .build()

        schedule(
            SequentialCommandGroup(
                InstantCommand({intake.servoLeft.position = 0.6
                    intake.servoRight.position = 0.6}),
                TrajectoryFollowerCommand(drive, when(spike.position){
                    SpikeProcessor.Position.LEFT -> spikeLeft
                    SpikeProcessor.Position.CENTER -> spikeMiddle
                    SpikeProcessor.Position.RIGHT -> spikeRight
                }),
                TrajectoryFollowerCommand(drive, deposit),/*
                ParallelCommandGroup(
                    TrajectoryFollowerCommand(drive, deposit),
                    DepositCmd(lift, arm, wrist, 3.0)
                ),
                InstantCommand({intake.servoLeft.position = 0.675
                    intake.servoRight.position = 0.675}),
                WaitCommand(300),
                HomeCmd(lift, arm, wrist),
                TrajectoryFollowerCommand(drive, park)*/

                InstantCommand({projection(when(spike.position){
                    SpikeProcessor.Position.LEFT -> 1
                    SpikeProcessor.Position.CENTER -> 2
                    SpikeProcessor.Position.RIGHT -> 3
                })})
            )
        )
    }


    fun projection(target: Int){
        Log.d("cvPose", "init cv")
        val startPos = Pose2d(aprilTag.getPose(target)!!.position+Vector2d(
            AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(target).fieldPosition.get(0).toDouble(),
            AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(target).fieldPosition.get(1).toDouble(),
        ), 0.0)
        Log.d("cvPose", startPos.toString())
        val drive = MecanumDrive(hardwareMap, startPos)
        val t = TrajectoryBuilder(
            startPos, 1e-6, 0.0,
            drive.defaultVelConstraint, drive.defaultAccelConstraint, 0.25, 0.1
        ).strafeTo(Vector2d(
            AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(target).fieldPosition.get(0).toDouble()+10,
            AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(target).fieldPosition.get(1).toDouble(),
        )).build()[0]
        val dt = DisplacementTrajectory(t)
        val hc = HolonomicController(
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

            Log.d("cvPose", "Getting pose")
            cvPose = aprilTag.getPose(target)!!.position
            Log.d("cvPose", cvPose.toString())
            cvPose += Vector2d(
                AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(target).fieldPosition.get(0)
                    .toDouble(),
                AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(target).fieldPosition.get(1)
                    .toDouble(),
            )

            Log.d("cvPoseGlobal", cvPose.toString())

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

            val field = p.fieldOverlay()
            // We divide by 0.0254 to convert meters to inches
            // We divide by 0.0254 to convert meters to inches
            val translation = cvPose
            val rotation = drive.pose.heading.log()

            val robotRadius = 9.0

            field.strokeCircle(translation.x, translation.y, robotRadius)
            val arrowX: Double = cos(rotation) * robotRadius
            val arrowY: Double = sin(rotation) * robotRadius
            val x1 = translation.x + arrowX / 2
            val y1 = translation.y + arrowY / 2
            val x2 = translation.x + arrowX
            val y2 = translation.y + arrowY
            field.strokeLine(x1, y1, x2, y2)

            field.strokeCircle(
                AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(target).fieldPosition.get(0)
                .toDouble(),
                AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(target).fieldPosition.get(1)
                    .toDouble(), 2.0)


            dash.sendTelemetryPacket(p)
        }
        Log.d("cvPose", "Done")
        drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
    }
}
