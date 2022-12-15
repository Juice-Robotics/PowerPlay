package org.firstinspires.ftc.teamcode.commands;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot;

public class HighJunctionCommand extends SequentialCommandGroup {
    public HighJunctionCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.highPreset(false)),
                new WaitCommand(100),
                new InstantCommand(() -> robot.toggleClaw(false))
        );
    }
}
