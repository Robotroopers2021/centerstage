package org.firstinspires.ftc.teamcode.subsystems.wrist

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry


@Config
class Wrist(hardwareMap: HardwareMap, telemetry: Telemetry): SubsystemBase() {

    var wrist: Servo

    //Config values
    companion object{
        @JvmField var lowerPos = 0.0
        @JvmField var raisePos = 0.375
    }

    init {
        //TODO: Create code for wrist servo
        wrist = hardwareMap.get(Servo::class.java, "wrist")

        wrist.position = 0.11

        //TODO: Set directions for servos
        //armLeft.direction = Servo.Direction.REVERSE

    }

    fun setWristPos(pos : Double) {
        wrist.position = pos
    }

}