package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeCmd

class IntakeSequenceCmd(intake : Intake, arm: Arm) : SequentialCommandGroup() {
    init {
        addCommands(
            IntakeCmd(intake)
        )
    }
}