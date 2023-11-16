package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.Servo

@Config
@TeleOp(name = "❄🥶🧊❄")
class LiftTest : OpMode() {

//    private lateinit var servo1: Servo
//    private lateinit var servo2: Servo
    private lateinit var servo3 : Servo
    private lateinit var servo4 : Servo
    private lateinit var servo5 : Servo

    private fun servoControl(){
        if(gamepad1.dpad_up) {
//            servo1.position = 0.5
//            servo2.position = 0.5
            servo3.position = 0.5
            servo4.position = 0.5
            servo5.position = 0.5
        }
        if(gamepad1.dpad_down) {
//            servo1.position = 0.0
//            servo2.position = 0.0
            servo3.position = 0.0
            servo4.position = 0.0
            servo5.position = 0.0
        }
        }


    override fun init() {
//        servo1 = hardwareMap.get(Servo::class.java, "armRight");
//        servo2 = hardwareMap.get(Servo::class.java, "armLeft");
        servo3 = hardwareMap.get(Servo::class.java, "wrist");
        servo4 = hardwareMap.get(Servo::class.java, "claw1");
        servo5 = hardwareMap.get(Servo::class.java, "claw2");
    }

    override fun loop() {
        servoControl()
    }
}