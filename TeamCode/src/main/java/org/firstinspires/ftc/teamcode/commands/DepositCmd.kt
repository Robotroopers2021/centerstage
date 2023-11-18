package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCmd
import org.firstinspires.ftc.teamcode.subsystems.arm.WristCmd
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCmd

class DepositCmd(lift: Lift, arm: Arm, liftPos : Double, armPos : Double, wristPos : Double) : SequentialCommandGroup() {
    init {
        addCommands(
            ArmCmd(arm, armPos),
            WaitCommand(350),
            ParallelCommandGroup(
            LiftCmd(lift, liftPos),
            WristCmd(arm, wristPos)
            )
        )
    }
}