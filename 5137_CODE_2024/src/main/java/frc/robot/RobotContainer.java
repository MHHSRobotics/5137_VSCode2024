// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.*;
import frc.robot.Constants.*;
import frc.robot.Subsystems.*;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class RobotContainer {

  public static CommandPS4Controller musician;

  public Music music;

  public Music_Commands music_Commands;

  public RobotContainer() {
    musician = new CommandPS4Controller(0);

    music = new Music();

    music_Commands = new Music_Commands(music);

    configureBindings();
  }

  private void configureBindings() {
    musician.cross()
    .onTrue(music_Commands.toggleMusic());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
