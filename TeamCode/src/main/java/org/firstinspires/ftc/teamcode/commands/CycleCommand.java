package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class CycleCommand extends SequentialCommandGroup {
    public CycleCommand(Robot robot, Pose2d startPose, Robot.StarterStack level) {
        super(
                //STARTER STACK
                new InstantCommand(() -> robot.starterStackPreset(level)),
                new WaitCommand(2000),
                new InstantCommand(() -> robot.claw.toggle()),
                new WaitCommand(500),
                new InstantCommand(() -> robot.slides.runToPosition(-100)),

                // HIGH JUNCTION
                new TrajectorySequenceCommand(robot.drive.trajectorySequenceBuilder(startPose)
                        .setReversed(true)
                        .splineTo(new Vector2d(30, 10), 180)
                        .build()),
                new InstantCommand(() -> {
                    robot.slides.runToPreset(Levels.HIGH);
                    robot.v4b.setAngle(218);
                }),
                new WaitCommand(1000),
                new InstantCommand(() -> robot.claw.toggle()),
                new WaitCommand(1000),
                new InstantCommand(() -> {
                    robot.slides.runToPosition(-290);
                    robot.claw.setXRotation(5);
                }),
                new WaitCommand(100),
                new InstantCommand(() -> {
                    robot.v4b.runToPreset(Levels.GROUND);
                    robot.claw.setYRotation(0);
                }),
                new TrajectorySequenceCommand(robot.drive.trajectorySequenceBuilder(new Pose2d(30, 10, 180))
                        .setReversed(false)
                        .splineTo(new Vector2d(60,11), 0)
                        .build())

        );
    }

    public Pose2d end() {
        return new Pose2d(30, 10, 180);
    }
}
