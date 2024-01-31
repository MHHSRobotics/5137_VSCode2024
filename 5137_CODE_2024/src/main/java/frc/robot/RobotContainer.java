// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import frc.robot.Commands.*;
import frc.robot.Subsystems.*;
<<<<<<< HEAD
=======

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
>>>>>>> 2da6360f20fe863bc220c7b6615b8312bdd0cb09
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
public class RobotContainer {
<<<<<<< HEAD
  public CommandPS4Controller driver;
  public CommandPS4Controller operator;
  private Intake intake; 
  private Intake_Commands intake_Commands;
  private Shooter shooter;
=======
  public static CommandPS4Controller driver;
  public static CommandPS4Controller operator;

  private Swerve swerve;

  private Swerve_Commands swerve_Commands;
>>>>>>> 2da6360f20fe863bc220c7b6615b8312bdd0cb09

  public RobotContainer() {
    driver = new CommandPS4Controller(0);
    operator = new CommandPS4Controller(1);
<<<<<<< HEAD
    intake = new Intake();
    shooter = new Shooter();
    intake_Commands = new Intake_Commands(intake, shooter);
    configureBindings();
  }



  private void configureBindings() {
    operator.square()
    .onTrue(intake_Commands.intakeForward())
    .onFalse(intake_Commands.toStop());
    operator.circle()
    .onTrue(intake_Commands.intakeReverse())
    .onFalse(intake_Commands.toStop());
    operator.cross()
    .onTrue(intake_Commands.continuousIntake());
    operator.R1()
    .onTrue(intake_Commands.shootDefault())
    .onFalse(intake_Commands.stop());
    operator.L1()
    .onTrue(intake_Commands.launch());
=======

    swerve = new Swerve(new File(Filesystem.getDeployDirectory(),"swerve"));

    swerve_Commands = new Swerve_Commands(swerve);

    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(swerve_Commands.drive(
      () -> driver.getLeftX(),
      () -> -driver.getLeftY(),
      () -> driver.getRightX(),
      () -> !driver.L1().getAsBoolean()
    ));
>>>>>>> 2da6360f20fe863bc220c7b6615b8312bdd0cb09
  }

  public Command getAutonomousCommand() {
    return swerve_Commands.runAuto("StartToMid");
  }
}