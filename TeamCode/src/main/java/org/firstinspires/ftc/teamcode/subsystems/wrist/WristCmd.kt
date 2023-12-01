package org.firstinspires.ftc.teamcode.subsystems.wrist

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs

class WristCmd(val wrist: Wrist, val pos : Double): CommandBase() {
    val timer = ElapsedTime()
    val startAnalog = wrist.position
    val startPos = wrist.position
    init {
        addRequirements(wrist)
    }
    override fun initialize() {
        wrist.setWristPos(pos)
        timer.reset()
    }

    override fun isFinished(): Boolean {
        return abs(abs(startAnalog-wrist.position) /355- abs(startPos-pos)) <0.05
        //return timer.milliseconds() > 250 //abs(arm.wrist.position - pos) < 0.01
    }
}