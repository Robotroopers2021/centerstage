package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeCmd
import org.firstinspires.ftc.teamcode.subsystems.wrist.Wrist
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristCmd
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristConstants

class IntakeSequenceCmd(intake: Intake, wrist: Wrist) : SequentialCommandGroup() {
    init {
        addCommands(
            ParallelCommandGroup(
                WristCmd(wrist, WristConstants.intakePosition),
                IntakeCmd(intake)
            ),
            WaitCommand(350),
            WristCmd(wrist, WristConstants.zeroPosition)
        )
    }
}