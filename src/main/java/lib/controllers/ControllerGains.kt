package lib.controllers

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward

data class ControllerGains(
    var kP: Double = 0.0,
    var kI: Double = 0.0,
    var kD: Double = 0.0,
    var kS: Double = 0.0,
    var kV: Double = 0.0,
    var kA: Double = 0.0,
    var kG: Double = 0.0,
) {
    fun buildPIDController() = PIDController(kP, kI, kD)

    fun buildSimpleFeedforward() = SimpleMotorFeedforward(kS, kV, kA)

    fun buildArmFeedforward() = ArmFeedforward(kS, kG, kV, kA)

    fun buildElevatorFeedforward() = ElevatorFeedforward(kS, kG, kV, kA)
}
