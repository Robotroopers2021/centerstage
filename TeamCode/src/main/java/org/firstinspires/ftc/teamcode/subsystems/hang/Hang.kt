package org.firstinspires.ftc.teamcode.subsystems.hang

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap

@Config
class Hang(hardwareMap: HardwareMap): SubsystemBase() {
    var hang: DcMotorEx
    var limit: DigitalChannel

    enum class opModeType{
        Autonomous,
        TeleOp
    }
    init{
        hang = hardwareMap.get(DcMotorEx::class.java, "liftLead")
        hang.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        hang.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        limit = hardwareMap.get(DigitalChannel::class.java, "hangLimit")
    }

    fun hangUp(){
        while(!limit.state){
            hang.power = 0.4
        }
    }
}