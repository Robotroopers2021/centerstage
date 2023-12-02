package org.firstinspires.ftc.teamcode.commands

import android.util.Log
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.PerpetualCommand
import com.arcrobotics.ftclib.command.RunCommand
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

class HomeCmd(lift: Lift, arm: Arm, wrist: Wrist) : SequentialCommandGroup() {
    init {
        val required = abs(WristConstants.intakePosition-WristConstants.middlePosition)
        val armLower = ArmCmd(arm, ArmConstants.intakePosition)
        addCommands(
            ParallelCommandGroup(
                WristCmd(wrist, WristConstants.middlePosition),
                LiftCmd(lift, LiftConstants.zeroPosition),
            ),
            ArmCmd(arm, ArmConstants.middlePosition),
            ParallelCommandGroup(
                WristCmd(wrist, WristConstants.bufferPosition),
                ArmCmd(arm, ArmConstants.bufferPosition),
            ) ,
//                    WaitUntilCommand { armLower.distTraveled / required > HomeConstants.waitUntil },
            WristCmd(wrist, WristConstants.intakePosition),
            ArmCmd(arm, ArmConstants.intakePosition)
//            WristCmd(wrist, 0.09),
//            ArmCmd(arm, ArmConstants.intakePosition),
//            WristCmd(wrist, WristConstants.intakePosition)
        )
    }
}