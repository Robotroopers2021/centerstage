package org.firstinspires.ftc.teamcode.tests.opmodes

import com.acmerobotics.dashboard.config.Config

@Config
object RedConstants {
    @JvmField var startX = 24-(180/25.4)
    @JvmField var startY = -(72-16.75)
    @JvmField var startH = Math.toRadians(270.0)

    @JvmField var spikeLeftX = 9.0
    @JvmField var spikeLeftY = -20.0

    @JvmField var spikeMiddleX = 12.0
    @JvmField var spikeMiddleY = -26.0

    @JvmField var spikeRightX = 15.0
    @JvmField var spikeRightY = -20.0

    /*@JvmField var depositX = 47.0
    @JvmField var depositY = 36.0*/

    @JvmField var depositX = 43.5
    @JvmField var depositRightY = -33.0
    @JvmField var depositCenterY = -31.0
    @JvmField var depositLeftY = -26.0

    @JvmField var parkX = 60.0
    @JvmField var parkY = -2.0
}