package org.firstinspires.ftc.teamcode.subsystems.arm

import android.util.Log
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs

class ArmCmd(val arm: Arm, val pos : Double): CommandBase() {
    val timer = ElapsedTime()
    val startAnalog = arm.position
    val startPos = arm.armLeft.position
    val distTraveled: Double
        get() = abs(startAnalog-arm.position)/355

    val distRequired: Double
        get() = abs(startPos-pos)
    init {
        addRequirements(arm)
    }
    override fun initialize() {
        arm.setArmPos(pos)
        timer.reset()
    }

    override fun isFinished(): Boolean {
        //return timer.milliseconds() > 35
        /*Log.d("ServoArm", abs(abs(startAnalog-arm.position)/355- abs(startPos-pos)).toString())
        Log.d("ServoArm", (abs(abs(startAnalog-arm.position)/355- abs(startPos-pos))<0.05).toString())*/
        Log.d("Arm", abs(distTraveled-distRequired).toString())
        return abs(distTraveled-distRequired)<0.05 || timer.milliseconds() > 1000
    }
}