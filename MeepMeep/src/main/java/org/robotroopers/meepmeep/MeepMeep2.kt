package org.robotroopers.meepmeep

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveShim


object MeepMeep2 {
    @JvmStatic
    fun main(args: Array<String>) {
        val meepMeep = MeepMeep(800)
        val myBot =
            DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(250.58748), Math.toRadians(250.58748), 11.86)
                .setDimensions(14.0, 14.0)
                .followTrajectorySequence { drive: DriveShim ->
                    drive.trajectorySequenceBuilder(
                        Pose2d(
                            11.0,
                            62.5,
                            Math.toRadians(90.0)
                        )
                    )
                        .setReversed(true)
                        .lineToLinearHeading(Pose2d(11.0, 35.5, Math.toRadians(180.0)))
                        //left spike
//                        .splineToSplineHeading(Pose2d(11.0, 35.5, Math.toRadians(0.0)), Math.toRadians(90.0))
                        //right spike
//                        .splineToSplineHeading(Pose2d(11.0, 35.5, Math.toRadians(180.0)), Math.toRadians(90.0))
                        .lineToLinearHeading(Pose2d(48.0, 35.5, Math.toRadians(180.0)))
                        .setReversed(false)
                        .splineToConstantHeading(Vector2d(24.0, 11.5), Math.toRadians(180.0))
                        .splineToConstantHeading(Vector2d(0.0, 11.5), Math.toRadians(180.0))
                        .splineToConstantHeading(Vector2d(-60.0, 11.5), Math.toRadians(180.0))
                        .setReversed(true)
                        .splineToConstantHeading(Vector2d(0.0, 11.5), Math.toRadians(0.0))
                        .splineToConstantHeading(Vector2d(24.0, 11.5), Math.toRadians(0.0))
                        .splineToConstantHeading(Vector2d(48.0, 35.5), Math.toRadians(0.0))
                        .build()
                }
        meepMeep.setBackground(Background.FIELD_CENTERSTAGE_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start()
    }
}