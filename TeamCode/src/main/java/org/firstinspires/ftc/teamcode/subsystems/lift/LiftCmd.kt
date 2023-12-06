package org.firstinspires.ftc.teamcode.subsystems.lift

import android.util.Log
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.ProfiledPIDCommand
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime

//TODO: Make the input parameter number of pixels high
@Config
class LiftCmd(val lift: Lift, val pos: Double): ProfiledPIDCommand(
    pidController,
    {lift.liftLeadMotor.currentPosition.toDouble()/(LiftConstants.ticksPerInch)},
    pos,
    { output: Double, state: TrapezoidProfile.State -> lift.liftLeadMotor.power = output+ feedforward.calculate(state.velocity)
        lift.liftSecondMotor.power = lift.liftLeadMotor.power
        targetPos =state.position},
    lift
){
    init {
        addRequirements(lift)
    }

    val timer = ElapsedTime()
    override fun initialize() {
        pidController.setTolerance(0.05)
        Log.d("lift", "init command")
        targetPos = pos
        Log.d("lift", "Changed target to $pos")
        timer.reset()
    }
    companion object{
        var constraints = TrapezoidProfile.Constraints(LiftConstants.maxVel, LiftConstants.maxAccel)
        var pidController = ProfiledPIDController(LiftConstants.kp, LiftConstants.ki, LiftConstants.kd, constraints)
        var feedforward = ElevatorFeedforward(LiftConstants.ks, LiftConstants.kg, LiftConstants.kv, LiftConstants.ka)
        var targetPos = 0.0
    }

    override fun isFinished(): Boolean {
//        if (timer.milliseconds()>=2000)
//            return true
//        if(lift.liftLimit.state && pos==0.0) {
//            Log.d("Lift", "Hit Limit")
//            return true
//        }
        Log.d("Lift", "PID ${pidController.atGoal()}")
        Log.d("Lift", "Limit ${lift.liftLimit.state}")
        Log.d("Lift", "Timer ${timer.milliseconds()>2000}")

        return pidController.atGoal() || lift.liftLimit.state || timer.milliseconds()>2000
    }

    override fun end(interrupted: Boolean) {
        when(lift.liftLimit.state){
            true -> {lift.liftLeadMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                lift.liftSecondMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                Log.d("lift", "RESET ENCODER")
                lift.liftLeadMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                lift.liftSecondMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                lift.liftLeadMotor.power = 0.0
                lift.liftSecondMotor.power = 0.0}
            else -> {lift.liftLeadMotor.power = 0.1
                lift.liftSecondMotor.power = 0.1}
        }

        /*if(pos == LiftConstants.zeroPosition){
            lift.liftLeadMotor.power = -0.2
            lift.liftSecondMotor.power = -0.2
        }*/
    }
}