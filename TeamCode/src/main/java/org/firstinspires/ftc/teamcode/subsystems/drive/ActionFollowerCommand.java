package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;



public class ActionFollowerCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final Action action;

    public ActionFollowerCommand(MecanumDriveSubsystem drive, Action action) {
        this.drive = drive;
        this.action = action;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.followAction(action);
    }

    @Override
    public void execute() {
        drive.updatePoseEstimate();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }

}