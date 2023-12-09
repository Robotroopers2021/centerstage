package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCmd
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmConstants
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmConstants.spikeDepositPosition
import org.firstinspires.ftc.teamcode.subsystems.hang.Hang
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftConstants.spikeHeight
import org.firstinspires.ftc.teamcode.subsystems.wrist.Wrist
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristCmd
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristConstants

class SpikeDepositCmd(lift: Lift, arm: Arm, wrist: Wrist) : SequentialCommandGroup() {
    init {
        val armDeposit = ArmCmd(arm, ArmConstants.spikeDepositPosition)
        addCommands(
            DepositCmd(lift, arm, wrist, spikeHeight),
            ParallelCommandGroup(
                ArmCmd(arm, spikeDepositPosition),
                WristCmd(wrist, WristConstants.spikeDepositPosition)
            )
        )
    }
}