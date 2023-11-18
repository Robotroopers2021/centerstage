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

    //Config values
    companion object{
        @JvmField var lowerPos = 0.0
        @JvmField var raisePos = 0.375
    }

    init {
        armLeft = hardwareMap.get(Servo::class.java, "armLeft")
        armRight = hardwareMap.get(Servo::class.java, "armRight")

        armLeft.position = 0.02;
        armRight.position = 0.02;


        //TODO: Set directions for servos
        //armLeft.direction = Servo.Direction.REVERSE

        analog = hardwareMap.get(AnalogInput::class.java, "armAnalog")
    }

    fun raise(){
        armLeft.position = raisePos
        armRight.position = raisePos  //1-raisePos
    }

    fun lower(){
        armLeft.position = lowerPos
        armRight.position = lowerPos
    }

    fun setArmPos(pos : Double) {
        armLeft.position = pos
        armRight.position = pos
    }

    var position: Double = 0.0
        get() = analog.voltage/3.3*360
}