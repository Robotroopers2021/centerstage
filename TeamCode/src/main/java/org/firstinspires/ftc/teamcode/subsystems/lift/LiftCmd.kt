package org.firstinspires.ftc.teamcode.subsystems.lift

import android.util.Log
import com.arcrobotics.ftclib.command.ProfiledPIDCommand
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime

class LiftCmd(val lift: Lift, val pos: Double): ProfiledPIDCommand(
    pidController,
    {lift.liftLeadMotor.currentPosition.toDouble()/(1950.0/18.0)},
    pos,
    { output: Double, state: TrapezoidProfile.State -> lift.liftLeadMotor.power = output+ feedforward.calculate(state.velocity)
        lift.liftSecondMotor.power = lift.liftLeadMotor.power
        targetPos =state.position},
    lift
){
    val timer = ElapsedTime()
    override fun initialize() {
        addRequirements(lift)
        pidController.setTolerance(0.0)
        Log.d("lift", "init command")
        lift.targetPos = pos
        Log.d("lift", "Changed target to $pos")
        timer.reset()
    }
    companion object{
        var constraints = TrapezoidProfile.Constraints(300.0, 300.0)
        var pidController = ProfiledPIDController(0.3, 0.0, 0.005, constraints)
        var feedforward = ElevatorFeedforward(0.02, 0.02, 0.007, 0.0)
        var targetPos = 0.0
    }

    override fun isFinished(): Boolean {
        if (timer.milliseconds()>=2000)
            return true
        if(lift.liftLimit.state && pos==LiftPositions.HOME) {
            Log.d("Lift", "Hit encoder")
            return true
        }
        return pidController.atGoal()
    }

    override fun end(interrupted: Boolean) {
        when(pos){
            LiftPositions.HOME -> {lift.liftLeadMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                lift.liftSecondMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                Log.d("lift", "RESET ENCODER")
                lift.liftLeadMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                lift.liftSecondMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                lift.liftLeadMotor.power = 0.0
                lift.liftSecondMotor.power = 0.0}
            else -> {lift.liftLeadMotor.power = 0.1
                lift.liftSecondMotor.power = 0.1}
        }
    }
}