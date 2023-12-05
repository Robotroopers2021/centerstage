package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.command.WaitUntilCommand
import com.qualcomm.robotcore.hardware.DcMotor
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

class SaveLift(lift: Lift, arm: Arm, wrist: Wrist) : SequentialCommandGroup() {
    init {
        addCommands(
            WristCmd(wrist, WristConstants.depositPosition),
            ArmCmd(arm, ArmConstants.depositPosition),
            LiftCmd(lift, LiftConstants.depositHeight),
            InstantCommand({while(!lift.liftLimit.state){
                lift.liftLeadMotor.power = -0.6
                lift.liftSecondMotor.power = -0.6
            }
                lift.liftLeadMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                lift.liftSecondMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                lift.liftLeadMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                lift.liftSecondMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                lift.liftLeadMotor.power = 0.0
                lift.liftSecondMotor.power = 0.0}
            )
            )
    }
}