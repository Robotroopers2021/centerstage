package org.firstinspires.ftc.teamcode.tests.opmodes

import com.acmerobotics.dashboard.config.Config

@Config
object BlueConstants {
    @JvmField var startX = 24-(180/25.4)
    @JvmField var startY = 72-16.75
    @JvmField var startH = Math.toRadians(90.0)

    @JvmField var spikeLeftX = 15.0
    @JvmField var spikeLeftY = 20.0

    @JvmField var spikeMiddleX = 12.0
    @JvmField var spikeMiddleY = 26.0

    @JvmField var spikeRightX = 9.0
    @JvmField var spikeRightY = 20.0

    /*@JvmField var depositX = 47.0
    @JvmField var depositY = 36.0*/

    @JvmField var depositX = 46.0
    @JvmField var depositLeftY = 39.0
    @JvmField var depositCenterY = 31.0
    @JvmField var depositRightY = 26.0

    @JvmField var parkX = 60.0
    @JvmField var parkY = 5.0
}