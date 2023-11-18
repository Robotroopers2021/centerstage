package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCmd
import org.firstinspires.ftc.teamcode.subsystems.arm.WristCmd
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCmd

class HomeCmd(lift: Lift, arm: Arm) : SequentialCommandGroup() {
    init {
        addCommands(
            WristCmd(arm, 0.05),
            WaitCommand(350),
            ParallelCommandGroup(
                ArmCmd(arm, 0.215),
                LiftCmd(lift, 0.0)
            ),
            WristCmd(arm, 0.0),
            ArmCmd(arm, 0.115),
            WristCmd(arm, 0.03),
//            ArmCmd(arm, 0.0),
//            WristCmd(arm, 0.0)
        )
    }
}