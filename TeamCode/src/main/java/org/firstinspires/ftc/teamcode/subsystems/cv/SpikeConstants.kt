package org.firstinspires.ftc.teamcode.subsystems.cv

import com.acmerobotics.dashboard.config.Config
import org.opencv.core.Point

@Config
object SpikeConstants {
    @JvmField var lowerV0 = 0.0
    @JvmField var lowerV1 = 180.0
    @JvmField var lowerV2 = 0.0

    @JvmField var upperV0 = 255.0
    @JvmField var upperV1 = 255.0
    @JvmField var upperV2 = 110.0


    @JvmField var REGION1_TOPLEFT_ANCHOR_POINT_X = 0.0
    @JvmField var REGION1_TOPLEFT_ANCHOR_POINT_Y = 200.0
    @JvmField var REGION2_TOPLEFT_ANCHOR_POINT_X = 675.0
    @JvmField var REGION2_TOPLEFT_ANCHOR_POINT_Y = 250.0
    @JvmField var REGION3_TOPLEFT_ANCHOR_POINT_X = 1250.0
    @JvmField var REGION3_TOPLEFT_ANCHOR_POINT_Y = 200.0
}