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
    var wristAnalog: AnalogInput

    init {
        wrist = hardwareMap.get(Servo::class.java, "wrist")
        wristAnalog = hardwareMap.get(AnalogInput::class.java, "wristAnalog")
        wrist.position = WristConstants.intakePosition
    }

    fun setWristPos(pos : Double) {
        wrist.position = pos
    }

    var position: Double = 0.0
        get() = wristAnalog.voltage/3.3*360
}