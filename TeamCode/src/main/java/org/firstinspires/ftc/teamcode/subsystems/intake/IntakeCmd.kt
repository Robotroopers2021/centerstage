package org.firstinspires.ftc.teamcode.subsystems.intake

import android.util.Log
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ProfiledPIDCommand
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

//TODO: Make the input parameter number of pixels high
class IntakeCmd(val intake: Intake): CommandBase() {
    override fun initialize() {
        addRequirements(intake)
    }

    override fun isFinished(): Boolean {
        return intake.colorLeft.getDistance(DistanceUnit.INCH) < 1.0 && intake.colorRight.getDistance(DistanceUnit.INCH) < 1.0
    }

    override fun end(interrupted: Boolean) {
        intake.off()
        if(intake.colorRight.argb()==0) //Green
            intake.blinkinRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN)
        else if(intake.colorRight.argb()==1) //Purple
            intake.blinkinRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET)
        else if(intake.colorRight.argb()==2) //Yellow
            intake.blinkinRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW)
        else
            intake.blinkinRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE)

        //same for left
        if(intake.colorLeft.argb()==0) //Green
            intake.blinkinLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN)
        else if(intake.colorLeft.argb()==1) //Purple
            intake.blinkinLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET)
        else if(intake.colorLeft.argb()==2) //Yellow
            intake.blinkinLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW)
        else
            intake.blinkinLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE)
    }
}