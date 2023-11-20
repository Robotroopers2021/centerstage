package org.firstinspires.ftc.teamcode.subsystems.arm

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry


@Config
class Arm(hardwareMap: HardwareMap, telemetry: Telemetry): SubsystemBase() {
    var armLeft: Servo
    var armRight: Servo
    var analog: AnalogInput

    init {
        armLeft = hardwareMap.get(Servo::class.java, "armLeft")
        armRight = hardwareMap.get(Servo::class.java, "armRight")

        armLeft.position = ArmConstants.intakePosition
        armRight.position = ArmConstants.intakePosition

        analog = hardwareMap.get(AnalogInput::class.java, "armAnalog")
    }

    fun raise(){
        armLeft.position = ArmConstants.depositPosition
        armRight.position = ArmConstants.depositPosition
    }

    fun lower(){
        armLeft.position = ArmConstants.zeroPosition
        armRight.position = ArmConstants.zeroPosition
    }

    fun setArmPos(pos : Double) {
        armLeft.position = pos
        armRight.position = pos
    }

    var position: Double = 0.0
        get() = analog.voltage/3.3*360
}