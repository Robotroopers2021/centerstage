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
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.commands.DepositCmd
import org.firstinspires.ftc.teamcode.commands.HomeCmd
import org.firstinspires.ftc.teamcode.commands.IntakeSequenceCmd
import org.firstinspires.ftc.teamcode.commands.SaveLift
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCmd
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmConstants
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.commands.GamepadDrive
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeCmd
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCmd
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftConstants
import org.firstinspires.ftc.teamcode.subsystems.wrist.Wrist
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristCmd
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristConstants

@TeleOp
class TestOp : CommandOpMode() {
    lateinit var drive : MecanumDriveSubsystem
    lateinit var lift : Lift
    lateinit var intake : Intake
    lateinit var arm : Arm
    lateinit var wrist: Wrist

    override fun initialize() {
        var dashboard = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
        val gamepad1 = GamepadEx(gamepad1)
        val gamepad2 = GamepadEx(gamepad2)

        lift = Lift(hardwareMap, telemetry, Lift.opModeType.TeleOp)
        drive = MecanumDriveSubsystem(hardwareMap, Pose2d(0.0, 0.0, 0.0), false)
        intake = Intake(hardwareMap, telemetry)
        arm = Arm(hardwareMap, telemetry)
        wrist = Wrist(hardwareMap, telemetry)

        val intakeCmd = IntakeCmd(intake)
        val armLower = ArmCmd(arm, ArmConstants.zeroPosition)
        val armRaise = ArmCmd(arm, ArmConstants.depositPosition)


        schedule(HomeCmd(lift, arm, wrist))

        gamepad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(DepositCmd(lift, arm, wrist, 3.0))

//       gamepad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//           .toggleWhenPressed(DepositCmd(lift, arm, wrist, 10.0), HomeCmd(lift, arm, wrist))

        gamepad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(HomeCmd(lift, arm, wrist))

        gamepad1.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(InstantCommand({intake.servoLeft.position = 0.675
            intake.servoRight.position = 0.675}))

        gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(LiftCmd(lift, (lift.liftLeadMotor.currentPosition.toDouble()/(LiftConstants.ticksPerInch)) + 3.0))

        gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(LiftCmd(lift, (lift.liftLeadMotor.currentPosition.toDouble()/(LiftConstants.ticksPerInch)) - 3.0))

        gamepad2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(WristCmd(wrist, 0.0))

        gamepad2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(WristCmd(wrist, 0.5))

        gamepad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(ArmCmd(arm, 0.0))

        gamepad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(ArmCmd(arm, 0.5))

        gamepad2.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(SaveLift(lift, arm, wrist))

        Trigger{gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.0}
            .whileActiveContinuous(IntakeSequenceCmd(intake, wrist))
            .whenInactive(InstantCommand({intake.intake.power = 0.0}))

        Trigger{gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.0}
            .whileActiveContinuous(InstantCommand({intake.intake.power = -0.75}))
            .whenInactive(InstantCommand({intake.intake.power = 0.0}))

        schedule(GamepadDrive(drive, { gamepad1.leftY }, { gamepad1.leftX }, { gamepad1.rightX }))
        register(lift, intake, arm)
    }
    override fun run() {
        super.run()
        telemetry.addData("Arm Position", arm.position)
        telemetry.addData("Wrist Position", wrist.position)
        telemetry.addData("Lift Position", lift.liftLeadMotor.currentPosition)
        telemetry.addData("Lift Limit", lift.liftLimit.state)
        telemetry.update()
    }

}