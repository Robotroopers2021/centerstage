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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@Config
class Intake(hardwareMap: HardwareMap, telemetry: Telemetry): SubsystemBase() {
    var intake: DcMotorEx
    var colorLeft: RevColorSensorV3
    var colorRight: RevColorSensorV3
    var blinkinLeft: RevBlinkinLedDriver
    var blinkinRight: RevBlinkinLedDriver
    var mTelemetry: MultipleTelemetry
    var servoLeft: Servo
    var intakeDrop: Servo
    var servoRight: Servo

    init{
        mTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        intake = hardwareMap.get(DcMotorEx::class.java, "intake")
        //intake.direction = DcMotorSimple.Direction.REVERSE
        intake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intake.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        intake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        colorLeft = hardwareMap.get(RevColorSensorV3::class.java, "color1")
        colorRight = hardwareMap.get(RevColorSensorV3::class.java, "color2")

        intakeDrop = hardwareMap.get(Servo::class.java, "intakeDrop")
//
        blinkinLeft = hardwareMap.get(RevBlinkinLedDriver::class.java, "blinkinLeft")
        blinkinRight = hardwareMap.get(RevBlinkinLedDriver::class.java, "blinkinRight")
//
        servoLeft = hardwareMap.get(Servo::class.java, "claw1")
        servoRight = hardwareMap.get(Servo::class.java, "claw2")

        servoLeft.position = 0.675
        servoRight.position = 0.675

        blinkinRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW)
        blinkinLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW)
    }

    fun on(){
        intake.power = 0.75
        servoLeft.position = 0.675
        servoRight.position = 0.675;
    }

    fun off(){
        intake.power = 0.0
        servoLeft.position = 0.6
        servoRight.position = 0.6
    }

    fun drop(){
        intakeDrop.position = 0.0
    }

    override fun periodic() {
        if (colorRight.getDistance(DistanceUnit.INCH) < 0.5) {
            if (colorRight.argb() == 0) //Green
                blinkinRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN)
            else if (colorRight.argb() == 1) //Purple
                blinkinRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET)
            else if (colorRight.argb() == 2) //Yellow
                blinkinRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW)
            else
                blinkinRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE)
        }
        else{
            blinkinRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW)
        }

        //same for left
        if (colorLeft.getDistance(DistanceUnit.INCH) < 0.5) {
            if (colorLeft.argb() == 0) //Green
                blinkinLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN)
            else if (colorLeft.argb() == 1) //Purple
                blinkinLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET)
            else if (colorLeft.argb() == 2) //Yellow
                blinkinLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW)
            else
                blinkinLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE)
        }
        else{
            blinkinLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW)
        }
    }
}