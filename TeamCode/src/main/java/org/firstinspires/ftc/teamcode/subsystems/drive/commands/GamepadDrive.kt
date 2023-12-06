package org.firstinspires.ftc.teamcode.subsystems.drive.commands

import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.alphago.agDistanceLocalization.roadrunner.asUnitCircleHeading
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveSubsystem
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier


class GamepadDrive(
    drive: MecanumDriveSubsystem, leftY: DoubleSupplier,
    leftX: DoubleSupplier, rightX: DoubleSupplier,
    val autoStop: BooleanSupplier
) : CommandBase() {
    private val drive: MecanumDriveSubsystem
    private val leftY: DoubleSupplier
    private val leftX: DoubleSupplier
    private val rightX: DoubleSupplier
    override fun execute() {
        if(!autoStop.asBoolean) {
            drive.setDrivePower(
                PoseVelocity2d(
                    Vector2d(
                        leftY.asDouble,
                        -leftX.asDouble
                    ),
                    -rightX.asDouble
                )
            )
        }
        else{
            drive.setDrivePower(
                PoseVelocity2d(
                    Vector2d(
                        leftY.asDouble*0.25,
                        -leftX.asDouble*0.5
                    ),
                    -rightX.asDouble*0.25
                )
            )
        }
    }

    init {
        this.drive = drive
        this.leftX = leftX
        this.leftY = leftY
        this.rightX = rightX
        addRequirements(drive)
    }
}