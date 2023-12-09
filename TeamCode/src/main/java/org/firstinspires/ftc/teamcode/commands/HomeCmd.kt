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
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftConstants
import org.firstinspires.ftc.teamcode.subsystems.wrist.Wrist
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristCmd
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristConstants

class HomeCmd(lift: Lift, arm: Arm, wrist: Wrist) : SequentialCommandGroup() {
    init {
        addCommands(
            ParallelCommandGroup(
                WristCmd(wrist, WristConstants.middlePosition),
                LiftCmd(lift, LiftConstants.zeroPosition),
                SequentialCommandGroup(
                    WaitCommand(275),
                    ArmCmd(arm, ArmConstants.middlePosition),
                )
            ),
            WristCmd(wrist, WristConstants.bufferPosition),
            ArmCmd(arm, ArmConstants.intakePosition),
            WristCmd(wrist, WristConstants.intakePosition)
        )
    }
}