/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.PWMVictorSPX
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.CounterBase.EncodingType
import edu.wpi.first.wpilibj.PIDSourceType

import frc.robot.Constants

class MotorSubsystem : SubsystemBase() {
  /**
   * Creates a new MotorSubsystem.
   */

  private val motor: PWMVictorSPX = PWMVictorSPX(Constants.motorPort)
  private val encoder: Encoder = Encoder(0, 1, true, EncodingType.k4X)
  var encoderRate: Double = 0.0

  init {
    encoder.setDistancePerPulse(1.0)
    encoder.setPIDSourceType(PIDSourceType.kRate)
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  override fun periodic() {
    encoderRate = encoder.getRate()
    // println(encoderRate)
  }

  fun set(power: Double) = motor.set(power)

}
