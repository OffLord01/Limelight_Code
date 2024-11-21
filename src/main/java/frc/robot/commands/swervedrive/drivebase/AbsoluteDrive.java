// Copyright (c) FIRST and other WPILib contributors.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Dimensoes;
import frc.robot.Constants.Tracao;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * An example command that uses an example subsystem.
 */
public class AbsoluteDrive extends Command
{

  XboxController controle = new XboxController(0);
  //Nosso sensor de presença esta definido aqui 
    public static DigitalInput input = new DigitalInput(0);

    // Variáveis que guardam nossas funções do gamepad
    DoubleSupplier y;
    DoubleSupplier x;
    DoubleSupplier turn;

    // Objetos necessárias para acessar funções e variáveis
    SwerveSubsystem swerve;
    SwerveController controller;

    // Variáveis que guardam a translação e velocidade angular do swerve
    Translation2d translation;
    double angle;
    double omega;   

    public AbsoluteDrive(SwerveSubsystem swerve, DoubleSupplier y, DoubleSupplier x, DoubleSupplier turn) {
      // Aqui atribuimos as funções e subsistema
      this.y = y;
      this.x = x;
      this.turn = turn;
      this.swerve = swerve;
      controller = swerve.getSwerveController(); // Obtemos o controlador do swerve
      // Adiciona a tração como requerimento
      addRequirements(swerve);
    }

  @Override
  public void initialize()
  {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
  
    double xVelocity = y.getAsDouble() * Tracao.multiplicadorTranslacionalY;
      double yVelocity = x.getAsDouble() * Tracao.multiplicadorTranslacionalX;
      double angVelocity = turn.getAsDouble() * -Tracao.multiplicadorRotacional;


        translation = new Translation2d(xVelocity * Tracao.MAX_SPEED, yVelocity * Tracao.MAX_SPEED);
        omega = controller.config.maxAngularVelocity * angVelocity;
      
      if (Tracao.accelCorrection) {
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
            Dimensoes.LOOP_TIME, Dimensoes.ROBOT_MASS,
            List.of(Constants.Dimensoes.CHASSIS),
            swerve.getSwerveDriveConfiguration());
      } 

       swerve.drive(translation, omega, Tracao.fieldRelative);
      
    }

  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }


}
