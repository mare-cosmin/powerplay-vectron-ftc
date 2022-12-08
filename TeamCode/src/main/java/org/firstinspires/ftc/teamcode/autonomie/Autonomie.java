package org.firstinspires.ftc.teamcode.autonomie;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.teleop.HardwareTeleOp;

@Autonomous(name="BlueCarouselModificat")
public class Autonomie extends AutonomousInterface {

    @Override
    public void runRobotAuto() {
        robot.moveForward(0.9, 500);

    }
}