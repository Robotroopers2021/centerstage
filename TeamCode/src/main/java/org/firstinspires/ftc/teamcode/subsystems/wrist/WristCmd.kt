package org.firstinspires.ftc.teamcode.subsystems.wrist

import android.util.Log
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs

class WristCmd(val wrist: Wrist, val pos : Double): CommandBase() {
    val timer = ElapsedTime()
    val startAnalog = wrist.position
    val startPos = wrist.wrist.position

    val distTraveled: Double
        get() = abs(startAnalog-wrist.position)/355

    val distRequired: Double
        get() = abs(startPos-pos)
    init {
        addRequirements(wrist)
    }
    override fun initialize() {
        wrist.setWristPos(pos)
        timer.reset()
    }

    override fun isFinished(): Boolean {
        Log.d("Wrist", abs(distTraveled-distRequired).toString())
        return abs(distTraveled-distRequired)<0.05 || timer.milliseconds() > 1000
    }
}