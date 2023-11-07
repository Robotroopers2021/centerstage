package org.firstinspires.ftc.teamcode.subsystems.intake

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

@Config
class Intake(hardwareMap: HardwareMap, telemetry: Telemetry): SubsystemBase() {
    var intake: DcMotorEx
    var colorLeft: RevColorSensorV3
    var colorRight: RevColorSensorV3
    var blinkinLeft: RevBlinkinLedDriver
    var blinkinRight: RevBlinkinLedDriver
    var mTelemetry: MultipleTelemetry

    init{
        mTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        intake = hardwareMap.get(DcMotorEx::class.java, "intake")
        //intake.direction = DcMotorSimple.Direction.REVERSE
        intake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intake.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        intake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        colorLeft = hardwareMap.get(RevColorSensorV3::class.java, "colorLeft")
        colorRight = hardwareMap.get(RevColorSensorV3::class.java, "colorRight")

        blinkinLeft = hardwareMap.get(RevBlinkinLedDriver::class.java, "blinkinLeft")
        blinkinRight = hardwareMap.get(RevBlinkinLedDriver::class.java, "blinkinRight")
    }

    fun on(){
        intake.power = 1.0
    }

    fun off(){
        intake.power = 0.0
    }
}