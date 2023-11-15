package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.Pose2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm
import org.firstinspires.ftc.teamcode.subsystems.arm.LowerCmd
import org.firstinspires.ftc.teamcode.subsystems.arm.RaiseCmd
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.commands.GamepadDrive
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeCmd
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCmd
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftConstants

@TeleOp
class TestOp : CommandOpMode() {
    lateinit var drive : MecanumDriveSubsystem
    lateinit var lift : Lift
    lateinit var intake : Intake
    lateinit var arm : Arm

    override fun initialize() {
        var dashboard = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
        val gamepad1 = GamepadEx(gamepad1)
        val gamepad2 = GamepadEx(gamepad2)

        lift = Lift(hardwareMap, telemetry)
        drive = MecanumDriveSubsystem(MecanumDrive(hardwareMap, Pose2d(0.0,0.0,0.0)), false)
        intake = Intake(hardwareMap, telemetry)
        arm = Arm(hardwareMap, telemetry)
//        val armLower = LowerCmd(arm)
//        val armRaise = RaiseCmd(arm)
        val intakeCmd = IntakeCmd(intake)

        gamepad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(LiftCmd(lift, LiftConstants.depositHeight))

        gamepad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(LiftCmd(lift, 0.0))

        gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(LowerCmd(arm))
//
//        gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//            .whenPressed(armLower)

        Trigger{gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.0}
            .whileActiveContinuous(InstantCommand({intake.intake.power = 0.75}))
            .whenInactive(InstantCommand({intake.intake.power = 0.0}))

        Trigger{gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.0}
            .whileActiveContinuous(InstantCommand({intake.intake.power = -0.75}))
            .whenInactive(InstantCommand({intake.intake.power = 0.0}))

        schedule(GamepadDrive(drive, { gamepad1.leftY }, { gamepad1.leftX }, { gamepad1.rightX }))
        register(lift, intake, arm)
    }
    override fun run() {
        super.run()
        //stelemetry.addData("Arm Position", arm.position)
        telemetry.addData("Lift Position", lift.liftLeadMotor.currentPosition)
        telemetry.update()
    }

}