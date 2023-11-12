package org.firstinspires.ftc.teamcode.subsystems.lift

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

@Config
class Lift(hardwareMap: HardwareMap, telemetry: Telemetry): SubsystemBase() {
    var liftLeadMotor: DcMotorEx
    var liftSecondMotor: DcMotorEx
    var liftLimit: DigitalChannel
    var mTelemetry: MultipleTelemetry

    init{
        mTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        liftLeadMotor = hardwareMap.get(DcMotorEx::class.java, "liftLead")
        liftSecondMotor = hardwareMap.get(DcMotorEx::class.java, "lift2")
        liftSecondMotor.direction = DcMotorSimple.Direction.REVERSE
        liftLeadMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        liftSecondMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        liftLimit = hardwareMap.get(DigitalChannel::class.java, "liftLimit")
        liftLeadMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftSecondMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftLeadMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        liftSecondMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        mTelemetry.addData("Lift Pos", liftLeadMotor.currentPosition )
        mTelemetry.addData("Lift Target Pos", LiftCmd.targetPos*(LiftConstants.ticksPerInch))
        mTelemetry.update()
    }

    override fun periodic(){
        mTelemetry.addData("Lift Pos", liftLeadMotor.currentPosition)
        mTelemetry.addData("Lift Target Pos", LiftCmd.targetPos*(LiftConstants.ticksPerInch))
        mTelemetry.addData("Lift Power", liftLeadMotor.power)
        mTelemetry.update()
    }
}