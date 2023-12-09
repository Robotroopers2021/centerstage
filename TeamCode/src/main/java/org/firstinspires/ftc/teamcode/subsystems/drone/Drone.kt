package org.firstinspires.ftc.teamcode.subsystems.drone

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
class Drone(hardwareMap: HardwareMap, telemetry: Telemetry): SubsystemBase() {

    var drone: Servo
    var aim: Servo

    init {
        drone = hardwareMap.get(Servo::class.java, "drone")
        aim = hardwareMap.get(Servo::class.java, "aim")
        lock()
        lower()
    }

    fun lock(){
        drone.position = 0.0
    }

    fun shoot(){
        drone.position = 0.5
    }

    fun lower(){
        aim.position = 0.0
    }

    fun raise(){
        aim.position = 0.3
    }
}