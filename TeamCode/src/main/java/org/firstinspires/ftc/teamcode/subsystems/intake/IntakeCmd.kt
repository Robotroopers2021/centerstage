package org.firstinspires.ftc.teamcode.subsystems.intake

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class IntakeCmd(val intake: Intake): CommandBase() {
    init {
        addRequirements(intake)
    }

    override fun initialize() {
        intake.on()
    }

    override fun execute() {
        if(intake.colorLeft.getDistance(DistanceUnit.INCH)<0.5)
            intake.servoLeft.position = 0.25
        if(intake.colorRight.getDistance(DistanceUnit.INCH)<0.5)
            intake.servoRight.position = 0.25
    }

    override fun isFinished(): Boolean {
        return intake.colorLeft.getDistance(DistanceUnit.INCH) < 0.5 && intake.colorRight.getDistance(DistanceUnit.INCH) < 0.5
    }

    override fun end(interrupted: Boolean) {
        intake.off()
//        if(intake.colorRight.argb()==0) //Green
//            intake.blinkinRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN)
//        else if(intake.colorRight.argb()==1) //Purple
//            intake.blinkinRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET)
//        else if(intake.colorRight.argb()==2) //Yellow
//            intake.blinkinRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW)
//        else
//            intake.blinkinRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE)
//
//        //same for left
//        if(intake.colorLeft.argb()==0) //Green
//            intake.blinkinLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN)
//        else if(intake.colorLeft.argb()==1) //Purple
//            intake.blinkinLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET)
//        else if(intake.colorLeft.argb()==2) //Yellow
//            intake.blinkinLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW)
//        else
//            intake.blinkinLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE)
    }
}