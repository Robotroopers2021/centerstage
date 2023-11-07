/*package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.tests.opmodes.LiftConstants.depositHeight
import org.firstinspires.ftc.teamcode.tests.opmodes.LiftConstants.liftkd
import org.firstinspires.ftc.teamcode.tests.opmodes.LiftConstants.liftkf
import org.firstinspires.ftc.teamcode.tests.opmodes.LiftConstants.liftki
import org.firstinspires.ftc.teamcode.tests.opmodes.LiftConstants.liftkp
import org.firstinspires.ftc.teamcode.tests.opmodes.LiftConstants.ticksPerInch
import org.firstinspires.ftc.teamcode.tests.opmodes.MecanumDrive

@Config
@TeleOp(name = "❄🥶🧊❄")
class LiftTest : OpMode() {

    private lateinit var drive: SampleMecanumDrive
    private lateinit var lift1: DcMotorEx
    private lateinit var lift2: DcMotorEx

    private var liftController = PIDFController(PIDCoefficients(liftkp, liftki, liftkd))

    private var targetTicks = 0.0;
    var output = 0.0
    var pidOutput = 0.0
    private fun liftControl(){
        when {
            gamepad1.left_bumper -> {
                moveLift(depositHeight)
            }
            gamepad1.right_bumper -> {
                moveLift(0.0);
            }
        }

        val currentPosition = (lift1.currentPosition).toDouble()

        pidOutput = liftController.update(currentPosition)
        output = liftkf + pidOutput

        lift1.power = output
        lift2.power = output

        val packet = TelemetryPacket()
        packet.put("target height", targetTicks/ticksPerInch)
        packet.put("output", output)
        packet.put("Current Position", currentPosition/ ticksPerInch)
        packet.put("target ticks", targetTicks)
        FtcDashboard.getInstance().sendTelemetryPacket(packet)
    }

    private fun moveLift(height : Double){
        targetTicks = height * ticksPerInch
        liftController.reset()
        liftController.targetPosition = targetTicks
    }

    override fun init() {
        drive = SampleMecanumDrive(hardwareMap)
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        lift1 = hardwareMap.get(DcMotorEx::class.java, "lift1");
        lift2 = hardwareMap.get(DcMotorEx::class.java, "lift2")
        lift1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
        lift2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
        lift1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        lift2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        lift2.direction = DcMotorSimple.Direction.REVERSE
        liftController.reset()
        liftController.targetPosition = 0.0
    }


    override fun loop() {
        liftControl()
    }




}*/