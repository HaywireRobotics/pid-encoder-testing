/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands

import frc.robot.subsystems.MotorSubsystem

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj.controller.PIDController

class MotorToRate(val targetRate: Double, val m_subsystem: MotorSubsystem) : CommandBase() {
  /**
   * Creates a new MotorToRate.
   *
   * @param m_subsystem The subsystem used by this command.
   */

  val pidController: PIDController
  
  init {
    addRequirements(m_subsystem)

    pidController = PIDController(0.0000125 * 0.45, 0.0000125 * 0.94, 0.000000175)
    // pidController = PIDController(0.000009, 0.0009, 0.000000035)
  }

  // Called when the command is initially scheduled.
  override fun initialize() = pidController.reset()

  // Called every time the scheduler runs while the command is scheduled.
  override fun execute() {
    useOutput(pidController.calculate(generateMeasurement(), generateSetpoint()))

    // println(pidController.positionError)
    println(m_subsystem.encoderRate)
  }
  // Called once the command ends or is interrupted.
  override fun end(interrupted: Boolean) = m_subsystem.set(0.0)

  // Returns true when the command should end.
  override fun isFinished(): Boolean = false

  fun useOutput(output: Double) {
    //m_subsystem.set(-maxOf(output, 0.005)-0.1)
    m_subsystem.set(-output - 0.5)
    // println(output)
  }

  fun generateMeasurement(): Double = m_subsystem.encoderRate

  fun generateSetpoint(): Double = targetRate
}
