// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import frc.robot.Commands.*;
import frc.robot.Subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
public class RobotContainer {
  public CommandPS4Controller driver;
  public CommandPS4Controller operator;

  private Intake intake; 
  private Shooter shooter;

    private Intake_Commands intake_Commands;
    private Shooter_Commands shooter_Commands;

  public RobotContainer() {
    driver = new CommandPS4Controller(0);
    operator = new CommandPS4Controller(1);

    intake = new Intake();
    shooter = new Shooter();

    intake_Commands = new Intake_Commands(intake);
    shooter_Commands = new Shooter_Commands(shooter);

    configureBindings();
  }



  private void configureBindings() {
    operator.R2()
    .onTrue(intake_Commands.intakeForward())
    .onFalse(intake_Commands.stop());

    operator.L2()
    .onTrue(intake_Commands.intakeReverse())
    .onFalse(intake_Commands.stop());

    operator.touchpad()
    .onTrue(new ParallelCommandGroup(shooter_Commands.shoot(), intake_Commands.intakeForward(1.0)))
    .onFalse(new ParallelCommandGroup(shooter_Commands.stop(), intake_Commands.stop()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}