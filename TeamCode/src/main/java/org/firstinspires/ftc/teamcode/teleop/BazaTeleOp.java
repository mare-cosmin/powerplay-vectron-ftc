package org.firstinspires.ftc.teamcode.teleop;

//import necessary ftc packages
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomie.HardwareAutonomie;

@TeleOp(name = "BazaTeleOp", group = "TeleOp")
public class BazaTeleOp extends OpMode {
    public ElapsedTime runtime = new ElapsedTime();
    public HardwareAutonomie robot = new HardwareAutonomie(hardwareMap);



    double x;
    double t = 0;

    double joyScale;

    public void init() {
    }

    public void init_loop() {
    }

    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

    }

}
