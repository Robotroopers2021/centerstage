package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx

@Config
@Autonomous
class EncoderTest : LinearOpMode() {
    @OptIn(ExperimentalStdlibApi::class)
    override fun runOpMode() {
        var par0 = OverflowEncoder(
            RawEncoder(
                hardwareMap.get<DcMotorEx>(
                    DcMotorEx::class.java,
                    "leftFront"
                )
            )
        )
        var par1 = OverflowEncoder(
            RawEncoder(
                hardwareMap.get<DcMotorEx>(
                    DcMotorEx::class.java,
                    "rightFront"
                )
            )
        )
        var perp = OverflowEncoder(
            RawEncoder(
                hardwareMap.get<DcMotorEx>(
                    DcMotorEx::class.java,
                    "rightBack"
                )
            )
        )

        var lastPar0Pos = par0.getPositionAndVelocity().position
        var lastPar1Pos = par1.getPositionAndVelocity().position
        var lastPerpPos = perp.getPositionAndVelocity().position

        waitForStart()
        while (opModeIsActive()){
            val par0PosVel = par0.getPositionAndVelocity()
            val par1PosVel = par1.getPositionAndVelocity()
            val perpPosVel = perp.getPositionAndVelocity()

            val par0PosDelta = par0PosVel.position - lastPar0Pos
            val par1PosDelta = par1PosVel.position - lastPar1Pos
            val perpPosDelta = perpPosVel.position - lastPerpPos
            telemetry.addData("Par 0: ", par0PosDelta)
            telemetry.addData("Par 1: ", par1PosDelta)
            telemetry.addData("Perp: ", perpPosDelta)
            telemetry.update()
            lastPar0Pos = par0PosVel.position
            lastPar1Pos = par1PosVel.position
            lastPerpPos = perpPosVel.position
        }
    }
}