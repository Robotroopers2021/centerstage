package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCmd
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCmd
import org.firstinspires.ftc.teamcode.subsystems.wrist.Wrist
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristCmd

class HomeCmd(lift: Lift, arm: Arm, wrist: Wrist) : SequentialCommandGroup() {
    init {
        addCommands(
            WristCmd(wrist, 0.05),
            WaitCommand(350),
            ParallelCommandGroup(
                ArmCmd(arm, 0.215),
                LiftCmd(lift, 0.0)
            ),
            WristCmd(wrist, 0.0),
            ParallelCommandGroup(
            ArmCmd(arm, 0.01),
            SequentialCommandGroup(
                WaitCommand(90),
                WristCmd(wrist, 0.11),
            )
            )
        )
    }
}