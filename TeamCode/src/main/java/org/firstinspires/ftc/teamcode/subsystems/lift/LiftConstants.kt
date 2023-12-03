package org.firstinspires.ftc.teamcode.subsystems.lift

import com.acmerobotics.dashboard.config.Config

@Config
object LiftConstants {
    @JvmField var kp = 0.3
    @JvmField var ki = 0.0
    @JvmField var kd = 0.0
    @JvmField var ks = 0.02
    @JvmField var kg = 0.2
    @JvmField var kv = 0.007;
    @JvmField var ka = 0.007;
    @JvmField var maxVel = 300.0
    @JvmField var maxAccel = 300.0
    @JvmField var ticksPerInch = 78.0
    @JvmField var depositHeight = 15.0
    @JvmField var zeroPosition = -1.0;
}