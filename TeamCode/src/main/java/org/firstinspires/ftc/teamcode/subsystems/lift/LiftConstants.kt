package org.firstinspires.ftc.teamcode.subsystems.lift

import com.acmerobotics.dashboard.config.Config

@Config
object LiftConstants {
    var liftkp = 0.01
    var liftki = 0.0
    var liftkd = 0.0
    var liftkf = 0.1
    var ticksPerInch = 78.0
    var depositHeight = 15.0
}