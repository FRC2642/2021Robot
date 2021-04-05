// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystem.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;


public class TimeDrive extends CommandBase {
  
  public static final SwerveDriveSubsystem drive = new SwerveDriveSubsystem();
  public double seconds;

  /** Creates a new TimeDrive. */
  public TimeDrive(double seconds) {
    addRequirements(drive);
    
    this.seconds = seconds;
    new WaitCommand(seconds).deadlineWith(new RunCommand(() -> drive.drive(.3, 0.0, 0.0), drive));

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
