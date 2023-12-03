package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCmd
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmConstants
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCmd
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftConstants
import org.firstinspires.ftc.teamcode.subsystems.wrist.Wrist
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristCmd
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristConstants
import kotlin.math.abs

class DepositCmd(lift: Lift, arm: Arm, wrist: Wrist, pos : Double) : SequentialCommandGroup() {
    init {
        val armDeposit = ArmCmd(arm, ArmConstants.depositPosition)
        addCommands(
            WristCmd(wrist, WristConstants.zeroPosition),
            ParallelCommandGroup(
                armDeposit,
//                    WaitUntilCommand { armDeposit.distTraveled / armDeposit.distRequired > 0.5 }), //Percent traveled
                WristCmd(wrist, WristConstants.middlePosition
                )
            ),
            ParallelCommandGroup(
            LiftCmd(lift, pos),
            WristCmd(wrist, WristConstants.depositPosition)
            )
        )
    }
}