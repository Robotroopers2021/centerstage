package org.firstinspires.ftc.teamcode.subsystems.wrist

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime

class WristCmd(val wrist: Wrist, val pos : Double): CommandBase() {
    val timer = ElapsedTime()
    init {
        addRequirements(wrist)
    }
    override fun initialize() {
        wrist.setWristPos(pos)
        timer.reset()
    }

    override fun isFinished(): Boolean {
        return timer.milliseconds() > 250 //abs(arm.wrist.position - pos) < 0.01
    }
}