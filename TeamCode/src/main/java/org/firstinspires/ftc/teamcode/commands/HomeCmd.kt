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
            ParallelCommandGroup(
                WristCmd(wrist, 0.05),
                LiftCmd(lift, 0.0),
                SequentialCommandGroup(
                    WaitCommand(275), //TODO need to tune delay
                    ArmCmd(arm, 0.215)
                )
            ),
            WristCmd(wrist, 0.0),
            ParallelCommandGroup(
            ArmCmd(arm, 0.03),
            SequentialCommandGroup(
                WaitCommand(65),
                WristCmd(wrist, 0.09),
            )
            )
        )
    }
}