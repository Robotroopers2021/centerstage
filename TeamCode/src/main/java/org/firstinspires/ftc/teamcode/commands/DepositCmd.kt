package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCmd
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCmd
import org.firstinspires.ftc.teamcode.subsystems.wrist.Wrist
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristCmd

class DepositCmd(lift: Lift, arm: Arm, wrist: Wrist, liftPos : Double, armPos : Double, wristPos : Double) : SequentialCommandGroup() {
    init {
        addCommands(
            WristCmd(wrist, 0.0),
            ArmCmd(arm, armPos),
            ParallelCommandGroup(
            LiftCmd(lift, liftPos),
            WristCmd(wrist, wristPos)
            )
        )
    }
}