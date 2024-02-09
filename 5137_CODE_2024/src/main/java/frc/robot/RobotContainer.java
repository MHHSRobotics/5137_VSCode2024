// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import frc.robot.Commands.*;
import frc.robot.Constants.Swerve_Constants;
import frc.robot.Subsystems.*;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {

  private CommandPS4Controller driver;
  private CommandPS4Controller operator;

  private Swerve swerve;
  private Arm arm;
  private Intake intake; 
  private Shooter shooter;
  private Vision vision;

  private Swerve_Commands swerve_Commands;
  private Arm_Commands arm_Commands;
  private Intake_Commands intake_Commands;

  public RobotContainer() {
    driver = new CommandPS4Controller(0);
    operator = new CommandPS4Controller(1);

    swerve = new Swerve(new File(Filesystem.getDeployDirectory(),"swerve"));
    intake = new Intake();
    shooter = new Shooter();
    vision = new Vision();
    arm = new Arm();

    swerve_Commands = new Swerve_Commands(swerve);
    intake_Commands = new Intake_Commands(intake, shooter);
    arm_Commands = new Arm_Commands(arm);
    vision.setDefaultCommand(new AddVisionMeasurement(vision));

    configureBindings();
  }



  private void configureBindings() {
    operator.R2()
    .onTrue(intake_Commands.intakeForward())
    .onFalse(intake_Commands.toStop());

    operator.L2()
    .onTrue(intake_Commands.intakeReverse())
    .onFalse(intake_Commands.toStop());
  
    operator.cross()
    .onTrue(arm_Commands.moveToSpeaker())
    .onFalse(arm_Commands.stopMoving());

    operator.square()
    .onTrue(arm_Commands.moveToAmp())
    .onFalse(arm_Commands.stopMoving());

    operator.triangle()
    .onTrue(arm_Commands.moveToStart())
    .onFalse(arm_Commands.stopMoving());

    operator.circle()
    .onTrue(arm_Commands.moveToIntake())
    .onFalse(arm_Commands.stopMoving());

    swerve.setDefaultCommand(swerve_Commands.drive(
      () -> MathUtil.applyDeadband(driver.getLeftX(), Swerve_Constants.LX_Deadband),
      () -> MathUtil.applyDeadband(-driver.getLeftY(), Swerve_Constants.LY_Deadband),
      () -> MathUtil.applyDeadband(driver.getRightX(), Swerve_Constants.RX_Deadband),
      () -> !driver.L1().getAsBoolean()
    ));

    driver.triangle()
    .onTrue(swerve_Commands.zeroGyro());

    driver.cross()
    .onTrue(swerve_Commands.aimAtTarget());


/* 
    driver.cross().onTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));

    driver.circle().onTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    driver.square().onTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

    driver.triangle().onTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    */
  }

  public Command getAutonomousCommand() {
    //return swerve_Commands.runAuto("StartToMid");
    return swerve_Commands.runAuto("StartToMid");
  }
}