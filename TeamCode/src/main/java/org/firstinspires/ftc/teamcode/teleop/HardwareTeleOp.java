package org.firstinspires.ftc.teamcode.teleop;

//import necessary ftc hardware packages
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareTeleOp {
    public ElapsedTime runtime = new ElapsedTime();

    //define motors
    public DcMotorEx motor_brat = null;
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;

    //define servos
    public Servo servo_brat = null;
    public Servo servo_brat2 = null;
    public Servo servo_gripper = null;

//    double COUNTS_CM_CHASSIS = 537.7 / (9.6 * 3.1415);
//    double COUNTS_CM_LIFT = 537.7 / ( 3.8 * 3.1415);

    //corectii




    public void initialize(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //defining other motors and servos
    }
    public void runToPositionCHASSIS(){
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPowerCHASSIS(double power){

        rightFront.setPower(power);
        rightBack.setPower(power);
        leftBack.setPower(power);
        leftFront.setPower(power);
    }
    public void setPowerZeroCHASSIS(){

        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
        leftFront.setPower(0);
    }
    public void stopAndResetEncodersCHASSIS(){

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void move(double power,double distance, int direction) {
        //moves with respect to direction:
        //if direction=1, moves forward
        //if direction=2, moves backward
        //if direction=3, moves left
        //if direction=4, moves right

        stopAndResetEncodersCHASSIS();

        /*
        int target =  (int)(distance * COUNTS_CM_CHASSIS * CORECTIE_FORWARD);
        if (direction == 1){
            leftFront.setTargetPosition(target);
            rightFront.setTargetPosition(target);
            leftBack.setTargetPosition(target);
            rightBack.setTargetPosition(target);
        }else if (direction == 2) {
            leftFront.setTargetPosition(-target);
            rightFront.setTargetPosition(-target);
            leftBack.setTargetPosition(-target);
            rightBack.setTargetPosition(-target);
        }else if (direction == 3) {
            leftFront.setTargetPosition(-target);
            rightFront.setTargetPosition(target);
            leftBack.setTargetPosition(target);
            rightBack.setTargetPosition(-target);
        }else if (direction == 4) {
            leftFront.setTargetPosition(target);
            rightFront.setTargetPosition(-target);
            leftBack.setTargetPosition(-target);
            rightBack.setTargetPosition(target);
        }else{
            telmetry.addData("Error","Direction not valid");
        }
        */


        runToPositionCHASSIS();

        runtime.reset();

        setPowerCHASSIS(power);

        setPowerZeroCHASSIS();

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopAndResetEncodersCHASSIS();
    }
    public void turn(double power, double distance, int direction) {
        //turns with respect to direction:
        //if direction=1, turns left
        //if direction=2, turns right

        stopAndResetEncodersCHASSIS();

        /*
        int target =  (int)(distance * COUNTS_CM_CHASSIS * CORECTIE_TURN);
        if (direction == 1){
            leftFront.setTargetPosition(-target);
            rightFront.setTargetPosition(target);
            leftBack.setTargetPosition(-target);
            rightBack.setTargetPosition(target);
        }else if (direction == 2) {
            leftFront.setTargetPosition(target);
            rightFront.setTargetPosition(-target);
            leftBack.setTargetPosition(target);
            rightBack.setTargetPosition(-target);
        }else{
            telemetry.addData("Error", "Invalid direction");
            telemetry.update();
        }
        */
        runToPositionCHASSIS();

        runtime.reset();

        setPowerCHASSIS(power);

        setPowerZeroCHASSIS();

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopAndResetEncodersCHASSIS();
    }

    //functii pentru celelalte motoare
}

