package org.firstinspires.ftc.teamcode.subsystems.lift

import com.acmerobotics.dashboard.config.Config

@Config
object ArmConstants {
    @JvmField var kp = 0.01
    @JvmField var ki = 0.0
    @JvmField var kd = 0.0
    @JvmField var kf = 0.1
    @JvmField var ticksPerInch = 78.0
    @JvmField var depositHeight = 15.0
}