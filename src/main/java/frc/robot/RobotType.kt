package frc.robot

import edu.wpi.first.wpilibj.RobotBase

object RobotType {
    val isReplay: Boolean = false

    val robot: Robot =
        if (RobotBase.isReal()) {
            Robot.BOUNTY
        } else if (isReplay) {
            Robot.REPLAY
        } else {
            Robot.SIM
        }

    enum class Robot {
        BOUNTY,
        SIM,
        REPLAY,
    }
}
