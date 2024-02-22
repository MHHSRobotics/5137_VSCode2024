// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import frc.robot.Commands.*;
import frc.robot.Constants.Swerve_Constants;
import frc.robot.Subsystems.*;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class RobotContainer {

  private CommandXboxController driver;
  private CommandPS4Controller operator;

  private Swerve swerve;
  private Arm arm;
  private Intake intake; 
  private Shooter shooter;
  private Vision vision;
  private LED led;

  private Swerve_Commands swerve_Commands;
  private Arm_Commands arm_Commands;
  private Intake_Commands intake_Commands;
  private Shooter_Commands shooter_Commands;
  private LED_Commands led_Commands;

  public RobotContainer() {
    driver = new CommandXboxController(0);
    operator = new CommandPS4Controller(1);

    swerve = new Swerve(new File(Filesystem.getDeployDirectory(),"swerve"));
    arm = new Arm(new File(Filesystem.getDeployDirectory(), "RobotConstants.json"));
    intake = new Intake();
    shooter = new Shooter();
    vision = new Vision();
    led = new LED();

    swerve_Commands = new Swerve_Commands(swerve, vision);
    arm_Commands = new Arm_Commands(arm);
    intake_Commands = new Intake_Commands(intake);
    shooter_Commands = new Shooter_Commands(shooter);
    vision.setDefaultCommand(new AddVisionMeasurement(vision, swerve));
    led_Commands = new LED_Commands(led);

    configureBindings();
  }

  private void configureBindings() {

    //Swerve Bindings

    /*
    driver.x()
    .onTrue(swerve.sysIdQuasisttatic(Direction.kForward));

    driver.y()
    .onTrue(swerve.sysIdQuasisttatic(Direction.kReverse));

    driver.a()
    .onTrue(swerve.sysIdDynamic(Direction.kForward));

    driver.b()
    .onTrue(swerve.sysIdDynamic(Direction.kReverse));*/
    
    
    swerve.setDefaultCommand(swerve_Commands.drive(
      () -> MathUtil.applyDeadband(driver.getLeftY(), Swerve_Constants.LY_Deadband),
      () -> MathUtil.applyDeadband(driver.getLeftX(), Swerve_Constants.LX_Deadband),
      () -> MathUtil.applyDeadband(driver.getRightX(), Swerve_Constants.RX_Deadband),
      () -> !driver.leftBumper().getAsBoolean()
    ));

    driver.a()
    .onTrue(swerve_Commands.aimAtSpeaker());

    driver.y()
    .onTrue(swerve_Commands.zeroGyro());
    
    

    // Arm Bindings

    arm.setDefaultCommand(arm_Commands.manualMove(() -> -operator.getLeftY()));

    operator.triangle()
    .onTrue(arm_Commands.moveToDefault());

    // Shooting Bindings

    operator.cross()

    .onTrue(
      new SequentialCommandGroup(
        shooter_Commands.shootSpeaker(),
        new WaitCommand(1),
        new ParallelCommandGroup(
          swerve_Commands.aimAtSpeaker(),
          arm_Commands.moveToSpeaker(
            new DoubleSupplier() {
              @Override
                public double getAsDouble() {
                return swerve.getDistanceToTarget();
              }
            }
          )
        ),
        new WaitCommand(1),
        intake_Commands.intakeForward()
      )
    )

    .onFalse(
      new ParallelCommandGroup(
        shooter_Commands.stop(),
        arm_Commands.moveToDefault(),
        intake_Commands.stop()
      )
    );

    operator.circle()

    .onTrue(
      new SequentialCommandGroup(
        arm_Commands.moveToAmp(),
        new WaitCommand(1),
        new ParallelCommandGroup(
          shooter_Commands.shootIntake(),
          intake_Commands.intakeForward()
        )
      )
    )

    .onFalse(
      new ParallelCommandGroup(
        shooter_Commands.stop(),
        arm_Commands.moveToDefault(),
        intake_Commands.stop()
      )
    );

    // Intake Bindings

    operator.R2()
    .onTrue(intake_Commands.intakeForward())
    .onFalse(intake_Commands.stop());
    
    operator.L2()
    .onTrue(intake_Commands.intakeReverse())
    .onFalse(intake_Commands.stop());

    new Trigger(new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return intake.objectInRange();
      }
    })
    .onTrue(intake_Commands.stop());
  }

  public Command getAutonomousCommand() {
    return swerve_Commands.runAuto("StartToMid");
  }
}