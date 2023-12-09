package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCmd
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmConstants
import org.firstinspires.ftc.teamcode.subsystems.hang.Hang
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCmd
import org.firstinspires.ftc.teamcode.subsystems.wrist.Wrist
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristCmd
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristConstants

class DepositCmd(lift: Lift, arm: Arm, wrist: Wrist, depositHeight: Double) : SequentialCommandGroup() {
    init {
        addCommands(
            WristCmd(wrist, WristConstants.zeroPosition),
            ParallelCommandGroup(
                ArmCmd(arm, ArmConstants.depositPosition),
                SequentialCommandGroup(
                    WaitCommand(25),
                    WristCmd(wrist, WristConstants.depositInternalPosition)
                )
            ),
            ParallelCommandGroup(
                LiftCmd(lift, depositHeight),
                WristCmd(wrist, WristConstants.depositPosition)
            )
        )
    }
}