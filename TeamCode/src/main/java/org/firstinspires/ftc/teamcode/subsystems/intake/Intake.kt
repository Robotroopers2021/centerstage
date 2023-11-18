package org.firstinspires.ftc.teamcode.subsystems.intake

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry

@Config
class Intake(hardwareMap: HardwareMap, telemetry: Telemetry): SubsystemBase() {
    var intake: DcMotorEx
    lateinit var colorLeft: RevColorSensorV3
    lateinit var colorRight: RevColorSensorV3
//    var blinkinLeft: RevBlinkinLedDriver
//    var blinkinRight: RevBlinkinLedDriver
    lateinit var mTelemetry: MultipleTelemetry
    lateinit var servoLeft: Servo
    lateinit var servoRight: Servo

    init{
        mTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        intake = hardwareMap.get(DcMotorEx::class.java, "intake")
        //intake.direction = DcMotorSimple.Direction.REVERSE
        intake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intake.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        intake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        colorLeft = hardwareMap.get(RevColorSensorV3::class.java, "color1")
        colorRight = hardwareMap.get(RevColorSensorV3::class.java, "color2")
//
//        blinkinLeft = hardwareMap.get(RevBlinkinLedDriver::class.java, "blinkinLeft")
//        blinkinRight = hardwareMap.get(RevBlinkinLedDriver::class.java, "blinkinRight")
//
        servoLeft = hardwareMap.get(Servo::class.java, "claw1")
        servoRight = hardwareMap.get(Servo::class.java, "claw2")

        servoLeft.position = 1.0
        servoRight.position = 1.0;
    }

    fun on(){
        intake.power = 0.75
        servoLeft.position = 1.0
        servoRight.position = 1.0;
    }

    fun off(){
        intake.power = 0.0
        servoLeft.position = 0.25
        servoRight.position = 0.25
    }
}