package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // TODO: If you experience poor performance, enable this flag
        // System.setProperty("sun.java2d.opengl", "true");

        // Creates field with 700 pixels and 60 fps
        MeepMeep meepMeep = new MeepMeep(700, 60);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(65, 50, Math.toRadians(180), Math.toRadians(180), 9)
                .setColorScheme(new ColorSchemeBlueDark())
                .setDimensions(12.5, 12.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -64, Math.toRadians(-90)))
                                .turn(Math.toRadians(40))
                                .lineTo(new Vector2d(-4, -39))
                                .turn(Math.toRadians(50))
                                .lineTo(new Vector2d(16, -64))
                                .lineTo(new Vector2d(45, -64))
                                .forward(-20)
                                .lineTo(new Vector2d(-4, -39))
                                .turn(Math.toRadians(-50))
                                .turn(Math.toRadians(50))
                                .lineTo(new Vector2d(16, -64))
                                .forward(10)
                                .build()
                );

//        // You can add a second bot if you want.
//        // You can comment this out as well if you only want one
//        RoadRunnerBotEntity bot2 = new DefaultBotBuilder(meepMeep)
//                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 13)
//                .setColorScheme(new ColorSchemeRedDark())
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(START_POSE2)
//                                //put ur trajectory here
//                                //for example
//                                .splineTo(new Vector2d(10, 10), 15)
//                                .build()
//                );
        meepMeep
                // Sets the field image
                .setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setTheme(new ColorSchemeBlueDark())
                .setBackgroundAlpha(1f)
                .addEntity(bot)
                //.addEntity(bot2) // comment out if you are only using one bot
                .start();
    }
}