package org.firstinspires.ftc.teamcode.subsystems.arm

import com.arcrobotics.ftclib.command.CommandBase
import kotlin.math.abs

class LowerCmd(val arm: Arm): CommandBase() {
    init {
        addRequirements(arm)
    }
    override fun initialize() {
        arm.lower()
    }

    override fun isFinished(): Boolean {
        return abs(arm.position - Arm.lowerPos) < 0.01
    }
}