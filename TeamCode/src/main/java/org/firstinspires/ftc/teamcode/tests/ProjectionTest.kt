package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.DisplacementTrajectory
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive.PARAMS

fun main(){
    var disp = 0.0

    var kinematics = MecanumKinematics(
        PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick)
    var dt = DisplacementTrajectory(TrajectoryBuilder(Pose2d(0.0, 0.0, Math.PI/2),
        1e-6, 0.0,
        MinVelConstraint(
            listOf(
                kinematics.WheelVelConstraint(PARAMS.maxWheelVel),
            AngularVelConstraint(PARAMS.maxAngVel)
        )
        ), ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel),
        0.25, 0.1
        ).lineToY(2.0).build()[0])

    disp = dt.project(Vector2d(10.0, 3.0), disp)
    //print(dt.length())
    print(disp)
    print(dt[disp].position.value().toString())
}