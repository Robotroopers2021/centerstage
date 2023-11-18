package org.firstinspires.ftc.teamcode.subsystems.arm

import com.arcrobotics.ftclib.command.CommandBase
import kotlin.math.abs

class ArmCmd(val arm: Arm, val pos : Double): CommandBase() {
    init {
        addRequirements(arm)
    }
    override fun initialize() {
        arm.setArmPos(pos)
    }

    override fun isFinished(): Boolean {
        return abs(arm.armLeft.position - pos) < 0.05
    }
}