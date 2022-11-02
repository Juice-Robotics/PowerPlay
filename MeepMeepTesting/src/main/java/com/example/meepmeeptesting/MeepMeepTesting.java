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
        MeepMeep meepMeep = new MeepMeep(800);

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(-360), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 58, Math.toRadians(-90)))

                               .splineTo(new Vector2d(-12, 56), Math.toRadians(-90))

                               .splineTo(new Vector2d(-3, 30), Math.toRadians(-75))

                                .back(5.0)

                                .splineTo(new Vector2d(-12, 13), Math.toRadians(-30))

                                .splineTo(new Vector2d(-61, 12), Math.toRadians(180))

                                .splineTo(new Vector2d(-23, 7), Math.toRadians(270))
                                .forward(5)
                                .splineTo(new Vector2d(-61, 12), Math.toRadians(0))

                                .splineTo(new Vector2d(-23, 7), Math.toRadians(270))
                                .back(5)
                                .splineTo(new Vector2d(-61, 12), Math.toRadians(0))

                                .splineTo(new Vector2d(-23, 7), Math.toRadians(270))
                                .forward(5)
                                .splineTo(new Vector2d(-61, 12), Math.toRadians(0))

                                .splineTo(new Vector2d(-23, 7), Math.toRadians(270))
                                .back(5)
                                .splineTo(new Vector2d(-61, 12), Math.toRadians(0))

                                .splineTo(new Vector2d(-23, 7), Math.toRadians(270))
                                .forward(5)
                                .splineTo(new Vector2d(-30, 11), Math.toRadians(270))
//
//                               .splineTo(new Vector2d(-28, 30), Math.toRadians(0))
//
//                                .splineTo(new Vector2d(-50, 50), Math.toRadians(100))

                                .build()
                );

        // Declare out second bot
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 58, Math.toRadians(-90)))
                                .strafeLeft(25)
                                .forward(35)
                                .turn(Math.toRadians(90))
                                .strafeRight(10)
                                .back(45)
                                //CYCLE 1
                                .forward(33)
                                .turn(Math.toRadians(-90))
                                .turn(Math.toRadians(90))
                                .back(33)

                                //CYCLE 2
                                .forward(33)
                                .turn(Math.toRadians(-90))
                                .turn(Math.toRadians(90))
                                .back(33)

                                //CYCLE 3
                                .forward(33)
                                .turn(Math.toRadians(-90))
                                .turn(Math.toRadians(90))
                                .back(33)

                                //CYCLE 4
                                .forward(33)
                                .turn(Math.toRadians(-90))
                                .turn(Math.toRadians(90))
                                .back(33)

                                //CYCLE 5
                                .forward(33)
                                .turn(Math.toRadians(-90))
                                .turn(Math.toRadians(90))
                                .back(10)


//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(myFirstBot)
//                .addEntity(mySecondBot)
              //  .addEntity(mySecondBot)
                .start();
    }
}
