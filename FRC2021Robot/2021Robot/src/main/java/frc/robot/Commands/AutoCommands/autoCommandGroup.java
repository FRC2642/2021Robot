// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.AutoCommands.TurnDrive;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystem.SwerveDriveSubsystem;
import frc.robot.Commands.AutoCommands.TimeDrive;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoCommandGroup extends SequentialCommandGroup {
  SwerveDriveSubsystem drive = new SwerveDriveSubsystem();

  /** Creates a new autoCommandGroup. */
  public autoCommandGroup() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
      addCommands(new TimeDrive(2.0),
                  new TurnDrive(45.0));
                }

}
