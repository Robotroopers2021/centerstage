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

    var wrist: Servo

    //Config values
    companion object{
        @JvmField var lowerPos = 0.0
        @JvmField var raisePos = 0.5
    }

    init {
        armLeft = hardwareMap.get(Servo::class.java, "servoLeft")
        armRight = hardwareMap.get(Servo::class.java, "servoRight")


        //TODO: Create code for wrist servo
        wrist = hardwareMap.get(Servo::class.java, "wrist")


        //TODO: Set directions for servos
        //armLeft.direction = Servo.Direction.REVERSE

        analog = hardwareMap.get(AnalogInput::class.java, "armAnalog")
    }

    fun raise(){
        armLeft.position = raisePos
        armRight.position = 1-raisePos
    }

    fun lower(){
        armLeft.position = lowerPos
        armRight.position = 1-lowerPos
    }

    val position: Double
        get() = analog.voltage/3.3*360
}