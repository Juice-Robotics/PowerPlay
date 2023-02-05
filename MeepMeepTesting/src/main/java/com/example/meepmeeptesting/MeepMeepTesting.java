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
                                drive.trajectorySequenceBuilder(new Pose2d(-35, 63, Math.toRadians(90)))
                                        .back(50)
                                        .addDisplacementMarker(1, ()-> {
//                                    robot.autoHigh(true);
//                                    robot.guide.setGuideDown();
                                        })
                                        .setReversed(true)
                                        .addDisplacementMarker(32, ()-> {
//                                    robot.slides.runToPreset(Levels.HIGH);
                                        })
                                        .splineTo(new Vector2d(-30.5,7), Math.toRadians(180 - 221.781))
                                        .addDisplacementMarker(54,()->{
//                                    robot.slides.runToPosition(-330);
                                        })
                                        .addTemporalMarker(2.3, ()->{
//                                    robot.claw.setClawOpen();
                                        })

                                        .addTemporalMarker(0, ()->{
//                    robot.autoLow(true);
                                        })
                                        .waitSeconds(0.5)
                                        .setReversed(false)
                                        .splineTo(new Vector2d(-57,8.3), Math.toRadians(180))
                                        .addTemporalMarker(1.4, ()->{
//                    robot.claw.setClawClose();
                                        })
                                        .addTemporalMarker(2.6, ()->{
//                    robot.slides.runToPreset(Levels.HIGH);
                                        })

                                        .setReversed(true)
                                        .splineTo(new Vector2d(-34,6), Math.toRadians(180 - 221.781))
                                        .addTemporalMarker(1, ()->{
//                    robot.autoHigh(true);
                                        })
                                        .waitSeconds(0.5)
                                        .addTemporalMarker(1.3, ()->{
//                    robot.slides.runToPosition(-280);
                                        })
                                        .addTemporalMarker(1.7,()->{
//                    robot.claw.setClawOpen();
                                        })
                                        .waitSeconds(0.2)

                                        .addTemporalMarker(0, ()->{
//                    robot.autoLow(true);
                                        })
                                        .waitSeconds(0.5)
                                        .setReversed(false)
                                        .splineTo(new Vector2d(-57,8.3), Math.toRadians(180))
                                        .addTemporalMarker(1.4, ()->{
//                    robot.claw.setClawClose();
                                        })
                                        .addTemporalMarker(2.6, ()->{
//                    robot.slides.runToPreset(Levels.HIGH);
                                        })

                                        .setReversed(true)
                                        .splineTo(new Vector2d(-34,6), Math.toRadians(180 - 221.781))
                                        .addTemporalMarker(1, ()->{
//                    robot.autoHigh(true);
                                        })
                                        .waitSeconds(0.5)
                                        .addTemporalMarker(1.3, ()->{
//                    robot.slides.runToPosition(-230);
                                        })
                                        .addTemporalMarker(1.7,()->{
//                    robot.claw.setClawOpen();
                                        })
                                        .waitSeconds(0.2)

                                        .addTemporalMarker(0, ()->{
//                    robot.autoLow(true);
                                        })
                                        .waitSeconds(0.5)
                                        .setReversed(false)
                                        .splineTo(new Vector2d(-57,8.3),  Math.toRadians(180))
                                        .addTemporalMarker(1.4, ()->{
//                    robot.claw.setClawClose();
                                        })
                                        .addTemporalMarker(2.6, ()->{
//                    robot.slides.runToPreset(Levels.HIGH);
                                        })

                                        .setReversed(true)
                                        .splineTo(new Vector2d(-34,6), Math.toRadians(180 - 221.781))
                                        .addTemporalMarker(1, ()->{
//                    robot.autoHigh(true);
                                        })
                                        .waitSeconds(0.5)
                                        .addTemporalMarker(1.3, ()->{
//                    robot.slides.runToPosition(-180);
                                        })
                                        .addTemporalMarker(1.7,()->{
//                    robot.claw.setClawOpen();
                                        })
                                        .waitSeconds(0.2)

                                        .addTemporalMarker(0, ()->{
//                    robot.autoLow(true);
                                        })
                                        .waitSeconds(0.5)
                                        .setReversed(false)
                                        .splineTo(new Vector2d(-57,8.3), Math.toRadians(180))
                                        .addTemporalMarker(1.4, ()->{
//                    robot.claw.setClawClose();
                                        })
                                        .addTemporalMarker(2.6, ()->{
//                    robot.slides.runToPreset(Levels.HIGH);
                                        })

                                        .setReversed(true)
                                        .splineTo(new Vector2d(-34,6), Math.toRadians(180 - 221.781))
                                        .addTemporalMarker(1, ()->{
//                    robot.autoHigh(true);
                                        })
                                        .waitSeconds(0.5)
                                        .addTemporalMarker(1.3, ()->{
//                    robot.slides.runToPosition(0);
                                        })
                                        .addTemporalMarker(1.7,()->{
//                    robot.claw.setClawOpen();
                                        })
                                        .waitSeconds(0.2)


                                        .addTemporalMarker(0, ()->{
//                    robot.autoLow(true);
                                        })
                                        .waitSeconds(0.5)
                                        .setReversed(false)
                                        .splineTo(new Vector2d(-57,8.3), Math.toRadians(180))
                                        .addTemporalMarker(1.4, ()->{
//                    robot.claw.setClawClose();
                                        })
                                        .addTemporalMarker(2.6, ()->{
//                    robot.slides.runToPreset(Levels.HIGH);
                                        })


                                        .setReversed(true)
                                        .splineTo(new Vector2d(-34,6), Math.toRadians(180 - 221.781))
                                        .addTemporalMarker(1, ()->{
//                    robot.autoHigh(true);
                                        })
                                        .waitSeconds(0.5)
                                        .addTemporalMarker(1.3, ()->{
//                    robot.slides.runToPosition(0);
                                        })
                                        .addTemporalMarker(1.7,()->{
//                    robot.claw.setClawOpen();
                                        })
                                        .waitSeconds(0.2)


                                        .build()
                );

        // Declare out second bot
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(35, 63, Math.toRadians(90)))
                                        .back(50)
                                        .addDisplacementMarker(1, ()-> {
//                                    robot.autoHigh(true);
//                                    robot.guide.setGuideDown();
                                        })
                                        .setReversed(true)
                                        .addDisplacementMarker(32, ()-> {
//                                    robot.slides.runToPreset(Levels.HIGH);
                                        })
                                        .splineTo(new Vector2d(30.5,7), 179.8)
                                        .addDisplacementMarker(54,()->{
//                                    robot.slides.runToPosition(-330);
                                        })
                                        .addTemporalMarker(2.3, ()->{
//                                    robot.claw.setClawOpen();
                                        })

                                        .addTemporalMarker(0, ()->{
//                    robot.autoLow(true);
                                        })
                                        .waitSeconds(0.5)
                                        .setReversed(false)
                                        .splineTo(new Vector2d(57,8.3), 0)
                                        .addTemporalMarker(1.4, ()->{
//                    robot.claw.setClawClose();
                                        })
                                        .addTemporalMarker(2.6, ()->{
//                    robot.slides.runToPreset(Levels.HIGH);
                                        })

                                        .setReversed(true)
                                        .splineTo(new Vector2d(34,6), 179.8)
                                        .addTemporalMarker(1, ()->{
//                    robot.autoHigh(true);
                                        })
                                        .waitSeconds(0.5)
                                        .addTemporalMarker(1.3, ()->{
//                    robot.slides.runToPosition(-280);
                                        })
                                        .addTemporalMarker(1.7,()->{
//                    robot.claw.setClawOpen();
                                        })
                                        .waitSeconds(0.2)

                                        .addTemporalMarker(0, ()->{
//                    robot.autoLow(true);
                                        })
                                        .waitSeconds(0.5)
                                        .setReversed(false)
                                        .splineTo(new Vector2d(57,8.3), 0)
                                        .addTemporalMarker(1.4, ()->{
//                    robot.claw.setClawClose();
                                        })
                                        .addTemporalMarker(2.6, ()->{
//                    robot.slides.runToPreset(Levels.HIGH);
                                        })

                                        .setReversed(true)
                                        .splineTo(new Vector2d(34,6), 179.8)
                                        .addTemporalMarker(1, ()->{
//                    robot.autoHigh(true);
                                        })
                                        .waitSeconds(0.5)
                                        .addTemporalMarker(1.3, ()->{
//                    robot.slides.runToPosition(-230);
                                        })
                                        .addTemporalMarker(1.7,()->{
//                    robot.claw.setClawOpen();
                                        })
                                        .waitSeconds(0.2)

                                        .addTemporalMarker(0, ()->{
//                    robot.autoLow(true);
                                        })
                                        .waitSeconds(0.5)
                                        .setReversed(false)
                                        .splineTo(new Vector2d(57,8.3), 0)
                                        .addTemporalMarker(1.4, ()->{
//                    robot.claw.setClawClose();
                                        })
                                        .addTemporalMarker(2.6, ()->{
//                    robot.slides.runToPreset(Levels.HIGH);
                                        })

                                        .setReversed(true)
                                        .splineTo(new Vector2d(34,6), 179.8)
                                        .addTemporalMarker(1, ()->{
//                    robot.autoHigh(true);
                                        })
                                        .waitSeconds(0.5)
                                        .addTemporalMarker(1.3, ()->{
//                    robot.slides.runToPosition(-180);
                                        })
                                        .addTemporalMarker(1.7,()->{
//                    robot.claw.setClawOpen();
                                        })
                                        .waitSeconds(0.2)

                                        .addTemporalMarker(0, ()->{
//                    robot.autoLow(true);
                                        })
                                        .waitSeconds(0.5)
                                        .setReversed(false)
                                        .splineTo(new Vector2d(57,8.3), 0)
                                        .addTemporalMarker(1.4, ()->{
//                    robot.claw.setClawClose();
                                        })
                                        .addTemporalMarker(2.6, ()->{
//                    robot.slides.runToPreset(Levels.HIGH);
                                        })

                                        .setReversed(true)
                                        .splineTo(new Vector2d(34,6), 179.8)
                                        .addTemporalMarker(1, ()->{
//                    robot.autoHigh(true);
                                        })
                                        .waitSeconds(0.5)
                                        .addTemporalMarker(1.3, ()->{
//                    robot.slides.runToPosition(0);
                                        })
                                        .addTemporalMarker(1.7,()->{
//                    robot.claw.setClawOpen();
                                        })
                                        .waitSeconds(0.2)


                                        .addTemporalMarker(0, ()->{
//                    robot.autoLow(true);
                                        })
                                        .waitSeconds(0.5)
                                        .setReversed(false)
                                        .splineTo(new Vector2d(57,8.3), 0)
                                        .addTemporalMarker(1.4, ()->{
//                    robot.claw.setClawClose();
                                        })
                                        .addTemporalMarker(2.6, ()->{
//                    robot.slides.runToPreset(Levels.HIGH);
                                        })


                                        .setReversed(true)
                                        .splineTo(new Vector2d(34,6), 179.8)
                                        .addTemporalMarker(1, ()->{
//                    robot.autoHigh(true);
                                        })
                                        .waitSeconds(0.5)
                                        .addTemporalMarker(1.3, ()->{
//                    robot.slides.runToPosition(0);
                                        })
                                        .addTemporalMarker(1.7,()->{
//                    robot.claw.setClawOpen();
                                        })
                                        .waitSeconds(0.2)


                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(myFirstBot)
                .addEntity(mySecondBot)
                //  .addEntity(mySecondBot)
                .start();
    }
}