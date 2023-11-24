package com.example.meeprr1;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRR1 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14.252, 14.252)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12.0, 62.5, Math.toRadians(90.0)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(17.0, 36.0), Math.toRadians(0.0))
                .waitSeconds(1.0)
                .strafeToLinearHeading(new Vector2d(47.0, 29.0), Math.toRadians(180.0))
//                        .setReversed(false)
//                .splineToConstantHeading(new Vector2d(24.0, 11.5), Math.toRadians(180.0))
//                .strafeToConstantHeading(new Vector2d(-60.0, 11.5))
//                        .setReversed(true)
//                .strafeToConstantHeading(new Vector2d(24.0, 11.5))
//                .splineToConstantHeading(new Vector2d(48.0, 35.5), Math.toRadians(90.0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}