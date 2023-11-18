package org.firstinspires.ftc.teamcode.subsystems.arm

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs

class ArmCmd(val arm: Arm, val pos : Double): CommandBase() {
    val timer = ElapsedTime()
    init {
        addRequirements(arm)
    }
    override fun initialize() {
        arm.setArmPos(pos)
        timer.reset()
    }

    override fun isFinished(): Boolean {
        return timer.milliseconds() > 350 //return abs(arm.armLeft.position - pos) < 0.05
    }
}