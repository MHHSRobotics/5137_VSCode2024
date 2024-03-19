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

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {

  private CommandPS4Controller driver;
  private CommandPS4Controller operator;
  //private CommandPS4Controller musician;

  private Swerve swerve;
  private Vision vision;
/* 
  private Arm arm;
  private Intake intake; 
  private Shooter shooter;
  private LED led;
  private Music music;
  */

  private Swerve_Commands swerve_Commands;
  /* 
  private Arm_Commands arm_Commands;
  private Intake_Commands intake_Commands;
  private Shooter_Commands shooter_Commands;
  private LED_Commands led_Commands;
  private Music_Commands music_Commands;
  */

  public RobotContainer() {
    driver = new CommandPS4Controller(0);
    //operator = new CommandPS4Controller(1);
    //musician = new CommandPS4Controller(2);

    swerve = new Swerve(new File(Filesystem.getDeployDirectory(),"swerve"));
    vision = new Vision();
/* 
    arm = new Arm(new File(Filesystem.getDeployDirectory(), "RobotConstants.json"));
    intake = new Intake();
    shooter = new Shooter();
    led = new LED();
    music = new Music();
    */

    swerve_Commands = new Swerve_Commands(swerve);
    vision.setDefaultCommand(new AddVisionMeasurement(vision, swerve));

    /* 
    arm_Commands = new Arm_Commands(arm);
    intake_Commands = new Intake_Commands(intake);
    shooter_Commands = new Shooter_Commands(shooter);
    led_Commands = new LED_Commands(led);
    music_Commands = new Music_Commands(music);
    

    NamedCommands.registerCommand("intake", 
        new ParallelCommandGroup(
          arm_Commands.moveToIntake(),
          intake_Commands.intakeForward()
        )
    );

    NamedCommands.registerCommand("default", arm_Commands.moveToTrap());

    NamedCommands.registerCommand("shoot",
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          arm_Commands.moveToSpeaker(
            new DoubleSupplier() {
              @Override
              public double getAsDouble() {
                return swerve.getDistanceToTarget();
              }
            }
          ),
          shooter_Commands.shootSpeaker()
        ),
        new WaitCommand(0.65),
        intake_Commands.intakeForward(),
        new WaitCommand(0.3),
        new ParallelCommandGroup(
          shooter_Commands.rest(),
          intake_Commands.stop()
        )
      )
    );

    NamedCommands.registerCommand("shoot_Delay",
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          arm_Commands.moveToSpeaker(
            new DoubleSupplier() {
              @Override
              public double getAsDouble() {
                return swerve.getDistanceToTarget();
              }
            }
          ),
          shooter_Commands.shootSpeaker()
        ),
        new WaitCommand(1.5),
        intake_Commands.intakeForward(),
        new WaitCommand(0.3),
        new ParallelCommandGroup(
          shooter_Commands.rest(),
          intake_Commands.stop()
        )
      )
    );

    */

    swerve.setUpPathPlanner();

    configureBindings();
  }

  private void configureBindings() {

  

    //TODO: Check if the fieldRelativeTrigger fixes loop ovverrun
    //Swerve Bindings

    
     swerve.setDefaultCommand(swerve_Commands.drive(
      () -> MathUtil.applyDeadband(driver.getLeftY(), Swerve_Constants.LY_Deadband),
      () -> MathUtil.applyDeadband(driver.getLeftX(), Swerve_Constants.LX_Deadband),
      () -> MathUtil.applyDeadband(getAllianceInvert()*driver.getRightX(), Swerve_Constants.RX_Deadband),
      () -> true
    ));
    
    driver.touchpad()
    .onTrue(swerve_Commands.zeroGyro());
    
    /* 
    driver.cross()
    .onTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
    driver.circle()
    .onTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    driver.square()
    .onTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    driver.triangle()
    .onTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    */

    driver.cross()
    .onTrue(swerve_Commands.driveToNote(
      () -> vision.getMetersToNote(),
      () -> vision.getRadiansToNote()
    ));
    /* 
    driver.cross()
    .onTrue(swerve_Commands.alignToSpeaker(true));
    */

    new Trigger(new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return Math.abs(driver.getRightX()) > Swerve_Constants.RX_Deadband;
      }
    })
    .onTrue(swerve_Commands.alignToSpeaker(false));

    
    driver.circle()
    .onTrue(swerve_Commands.driveToAmp());

    driver.square()
    .onTrue(swerve_Commands.driveToTrap());

    driver.triangle()
    .onTrue(swerve_Commands.zeroGyro());

    // Other Bindings

    driver.touchpad()
    .onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
    
    // Arm Bindings
/* 
    arm.setDefaultCommand(arm_Commands.manualMove(() -> -MathUtil.applyDeadband(operator.getLeftY(), 0.1)));

    // Shooting Bindings


   
    /* 
    operator.cross()
    .onTrue(
      new SequentialCommandGroup(
        shooter_Commands.shootSpeaker(),
        new WaitCommand(1),
        arm_Commands.moveToSpeaker(
          new DoubleSupplier() {
            @Override
              public double getAsDouble() {
              return swerve.getDistanceToTarget();
            }
          }
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
        shooter_Commands.shootIntake()
      )
    )
    .onFalse(
      new ParallelCommandGroup(
        shooter_Commands.stop(),
        arm_Commands.moveToDefault()
      )
    );

    operator.square()
    .onTrue(
      new SequentialCommandGroup(
        new ParallelCommandGroup(
        arm_Commands.moveToTrap(),
        shooter_Commands.shootSpeaker()
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

    // Intake Bindings & Stop Command

    operator.triangle()
    .onTrue(
      new ParallelCommandGroup(
        arm_Commands.moveToIntake(),
        intake_Commands.intakeForward()
      )
    )
    .onFalse(new ParallelCommandGroup(
        arm_Commands.moveToDefault(),
        intake_Commands.stop()
      )
    );

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
    .onTrue(
      new ParallelCommandGroup(
        intake_Commands.stop(),
        led_Commands.greenLedsOn()
      ))
    .onFalse(led_Commands.greenLedsOff());

    //Swerve Invert

    new Trigger(new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return (DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false);
      }
    })
    .onTrue(new InstantCommand(() -> swerve.motorInvert()));

    new Trigger(new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return (DriverStation.getAlliance().isPresent() ? true : false);
      }
    })
    .onTrue(new InstantCommand(() -> swerve.motorInvert()));

    new Trigger(new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return (DriverStation.isEnabled());
      }
    })
    .onTrue(new InstantCommand(() -> swerve.motorInvert()));
    */
  }
  
  public void configureMusic() {
    /* 
    musician.cross()
    .onTrue(music_Commands.tuningNote());

    musician.square()
    .onTrue(music_Commands.ConcertD());

    musician.circle()
    .onTrue(music_Commands.ConcertF());

    musician.triangle()
    .onTrue(music_Commands.ConcertA());
    */
  }

  public Command getAutonomousCommand() {
    return swerve_Commands.runAuto();
  }
  
  private double getAllianceInvert(){
    if(DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false)
    {
      return -1;
    }
    return 1;
  }

  public void autonomousInit() {
    swerve.autonomousInit();
  }
}