package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TrajectorySequenceCommand extends InstantCommand {
    public TrajectorySequenceCommand(TrajectorySequence trajectory) {
        super(
                () -> trajectory.start()
        );
    }
}
