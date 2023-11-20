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

    init {
        wrist = hardwareMap.get(Servo::class.java, "wrist")
        wrist.position = WristConstants.intakePosition
    }

    fun setWristPos(pos : Double) {
        wrist.position = pos
    }

}