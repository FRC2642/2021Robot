// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import frc.robot.Subsystem.SwerveDriveSubsystem;


public class FollowTrajectoryCommand extends CommandBase {
  private final EncoderFollowers[] followers;
  SwerveDriveSubsystem drive = new SwerveDriveSubsystem();

  /** Creates a new FollowTrajectoryCommand. */
  
  public FollowTrajectoryCommand(final Waypoint[] points) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    followers = drive.generateFollowers(points);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.startFollowing(followers);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0.0,0.0,0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drive.follow(followers);
  }
}
