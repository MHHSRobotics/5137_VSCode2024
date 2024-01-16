package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Robot;
import frc.robot.Constants.*;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;
    private CANcoder angleEncoder;

    private Rotation2d lastAngle;

    private final SparkPIDController driveController;
    private final SparkPIDController angleController;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Swerve_Constants.driveKS, Swerve_Constants.driveKV, Swerve_Constants.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        configAngleMotor();
        resetToAbsolute();

        /* Drive Motor Config */
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        configDriveMotor();
        lastAngle = new Rotation2d(0);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
          double percentOutput = desiredState.speedMetersPerSecond / Swerve_Constants.maxSpeed;
          driveMotor.set(percentOutput);
        } else {
          driveController.setReference(
              desiredState.speedMetersPerSecond,
              ControlType.kVelocity,
              0,
              driveFeedForward.calculate(desiredState.speedMetersPerSecond));
        }
      }
    
      private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle =
            (Math.abs(desiredState.speedMetersPerSecond) <= (Swerve_Constants.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle;
    
        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
      }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        integratedAngleEncoder.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(driveEncoder.getVelocity()/60.0, Swerve_Constants.wheelCircumference), 
            Rotation2d.fromRotations(integratedAngleEncoder.getPosition())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(driveEncoder.getPosition(), Swerve_Constants.wheelCircumference), 
            Rotation2d.fromRotations(integratedAngleEncoder.getPosition())
        );
    }

    private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        angleMotor.setSmartCurrentLimit(Swerve_Constants.angleCurrentLimit);
        angleMotor.setInverted(false);
        angleMotor.setIdleMode(Swerve_Constants.angleNeutralMode);
        integratedAngleEncoder.setPositionConversionFactor(Swerve_Constants.angleConversionFactor);
        angleController.setP(Swerve_Constants.angleKP);
        angleController.setI(Swerve_Constants.angleKI);
        angleController.setD(Swerve_Constants.angleKD);
        angleController.setFF(Swerve_Constants.angleKFF);
        angleMotor.enableVoltageCompensation(Swerve_Constants.voltageComp);
        angleMotor.burnFlash();
        resetToAbsolute();
      }
      private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        driveMotor.setSmartCurrentLimit(Swerve_Constants.driveContinuousCurrentLimit);
        driveMotor.setInverted(false); //TODO: Check if accurate
        driveMotor.setIdleMode(Swerve_Constants.driveNeutralMode);
        driveEncoder.setVelocityConversionFactor(Swerve_Constants.driveConversionVelocityFactor);
        driveEncoder.setPositionConversionFactor(Swerve_Constants.driveConversionPositionFactor);
        driveController.setP(Swerve_Constants.angleKP);
        driveController.setI(Swerve_Constants.angleKI);
        driveController.setD(Swerve_Constants.angleKD);
        driveController.setFF(Swerve_Constants.angleKFF);
        driveMotor.enableVoltageCompensation(Swerve_Constants.voltageComp);
        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
      }

   
}