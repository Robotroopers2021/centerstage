package org.firstinspires.ftc.teamcode.subsystems.arm

import com.arcrobotics.ftclib.command.CommandBase
import kotlin.math.abs

class WristCmd(val arm: Arm, val pos : Double): CommandBase() {
    init {
        addRequirements(arm)
    }
    override fun initialize() {
        arm.setWristPos(pos)
    }

    override fun isFinished(): Boolean {
        return abs(arm.wrist.position - pos) < 0.01
    }
}