package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RedUpperParkLeft", group = "Linear Opmode")
public class RedUpperParkLeft extends LinearOpMode {

    DcMotor R0, R2, L1, L3, CE1, CE2, SCM;
    Servo FNM;

    private ElapsedTime timer = new ElapsedTime();
    float power = (float) 0.7;
    private double counter = 0;
    private double increment = 0.09;


    public void setup() {

        R0 = hardwareMap.get(DcMotor.class, "R0");
        R2 = hardwareMap.get(DcMotor.class, "R2");
        L1 = hardwareMap.get(DcMotor.class, "L1");
        L3 = hardwareMap.get(DcMotor.class, "L3");
        CE1 = hardwareMap.get(DcMotor.class, "CE1");//cube eater 1 (right)
        CE2 = hardwareMap.get(DcMotor.class, "CE2");//cube eater 2 (left)
        SCM = hardwareMap.get(DcMotor.class, "SCM"); //seazer motor (misparaim)
        FNM = hardwareMap.get(Servo.class, "FNM");


        R0.setDirection(DcMotor.Direction.FORWARD);
        R2.setDirection(DcMotor.Direction.FORWARD);
        L1.setDirection(DcMotor.Direction.FORWARD);
        L3.setDirection(DcMotor.Direction.FORWARD);
        CE1.setDirection(DcMotor.Direction.FORWARD);
        CE2.setDirection(DcMotor.Direction.FORWARD);
        SCM.setDirection(DcMotor.Direction.FORWARD);
        FNM.setDirection(Servo.Direction.FORWARD);


        telemetry.addData("Stauts", "success!");
        telemetry.update();
    }


    public void setMotionEnginesMotorPower(float power) {
        R0.setPower(power);
        R2.setPower(power);
        L1.setPower(power);
        L3.setPower(power);
    }

    public void goFwd() {
        R0.setDirection(DcMotor.Direction.REVERSE);
        R2.setDirection(DcMotor.Direction.FORWARD);
        L1.setDirection(DcMotor.Direction.REVERSE);
        L3.setDirection(DcMotor.Direction.FORWARD);
        setMotionEnginesMotorPower(power);
    }

    public void drive(int dir, int cm, double power) {

        int target; // the original ticks of the motor ( see datasheet )
        int target_position; //the final position ( ticks )
        int error = 0; //the original ticks  divide by 3 ( 1993/3 = 665 )
        int error2; // the target position - original one circle 1328 ( 1993 -665 )
//        double power = 0.2; //power
        double acceleration = 0.01; // by how much the motor accelerate!
        target = cm * 45;//by the datasheet ( see goBilda web Motor 84 RPM )
        //45 ticks is 1 cm

        //set the direction of the motor
        // 1 --> FWD
        // 2 --> BWD
        //3 --> RIGHT
        //4 --> LEFT
        //5--> ROTATE LEFT
        //6--> ROTATE RIGHT
        switch (dir) {
            case 1: {
                L3.setDirection(DcMotor.Direction.REVERSE);
                L1.setDirection(DcMotor.Direction.FORWARD);
                R2.setDirection(DcMotor.Direction.REVERSE);
                R0.setDirection(DcMotor.Direction.FORWARD);
                break;
            }
            case 2: {
                L3.setDirection(DcMotor.Direction.FORWARD);
                L1.setDirection(DcMotor.Direction.REVERSE);
                R2.setDirection(DcMotor.Direction.FORWARD);
                R0.setDirection(DcMotor.Direction.REVERSE);
                break;
            }
            case 3: {
                L3.setDirection(DcMotor.Direction.FORWARD);
                L1.setDirection(DcMotor.Direction.REVERSE);
                R2.setDirection(DcMotor.Direction.REVERSE);
                R0.setDirection(DcMotor.Direction.FORWARD);
                break;
            }
            case 4: {
                L3.setDirection(DcMotor.Direction.REVERSE);
                L1.setDirection(DcMotor.Direction.FORWARD);
                R2.setDirection(DcMotor.Direction.FORWARD);
                R0.setDirection(DcMotor.Direction.REVERSE);
                break;

            }

            case 5: {
                R0.setDirection(DcMotor.Direction.REVERSE);
                R2.setDirection(DcMotor.Direction.REVERSE);
                L1.setDirection(DcMotor.Direction.REVERSE);
                L3.setDirection(DcMotor.Direction.REVERSE);
                break;
            }
            case 6: {
                R0.setDirection(DcMotor.Direction.FORWARD);
                R2.setDirection(DcMotor.Direction.FORWARD);
                L1.setDirection(DcMotor.Direction.FORWARD);
                L3.setDirection(DcMotor.Direction.FORWARD);
                break;
            }
        }

        //the mode of the motor
        //After using enncoder - do stop and reset
        L3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //mesure the encoder
        L3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //when power is 0 - do a breakes!
        L3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //supply power from the battery
        L3.setPower(power);
        L1.setPower(power);
        R2.setPower(power);
        R0.setPower(power);

        //calc the error - by ticks
        target_position = (L3.getCurrentPosition() + target) - error;
        // be safe with abs value
        //  error2 = Math.abs(target_position - 1328);
        // target_position = Math.abs(target_position - error2);


        //run to position
        while ((L3.getCurrentPosition() < target_position)) {


            //acceleration
            power = power + acceleration;
            if (power >= 1) {
                //dont over 1!
                // if the power is 1 --> keep it 1
                power = 1;
            }

            //keep update the power
            R2.setPower(power);
            L3.setPower(power);
            L1.setPower(power);
            R0.setPower(power);

            telemetry.addData("R2 ticks = ", R2.getCurrentPosition());
            telemetry.update();
        }

        //stop!
        L3.setPower(0);
        L1.setPower(0);
        R2.setPower(0);
        R0.setPower(0);

    }

    private int distanceToTurn(int degrees) {
        int wheel_diamater = 10;
        int len_betweeen_wheels = 46;
        int amountToTurn = (int) len_betweeen_wheels * degrees / (2 * wheel_diamater);
        //4 motors- divide by 4
        amountToTurn = amountToTurn / 4;
        return amountToTurn;
    }

    private void foundationCatchRelease(boolean down) {


        telemetry.addData("FNM position", FNM.getPosition());
        if (down){
            FNM.setDirection(Servo.Direction.REVERSE);
            FNM.setPosition(0.1);
//            counter = counter - increment;
        } else {
            FNM.setDirection(Servo.Direction.FORWARD);
            FNM.setPosition(0.02);
//            counter = counter + increment;
        }
        telemetry.addData("FNM position after", FNM.getPosition());
        telemetry.update();
        sleep(1000);


//        FNM.setDirection(Servo.Direction.FORWARD);

    }



    void test() {
        L1.setPower(1);
        sleep(2000);
        L1.setPower(0);

        L3.setPower(1);
        sleep(2000);
        L3.setPower(0);

        R2.setPower(1);
        sleep(2000);
        R2.setPower(0);

        R0.setPower(1);
        sleep(2000);
        R0.setPower(0);
    }


    //main
    @Override
    public void runOpMode() throws InterruptedException {

//        FoundationHandler foundationHandler = new FoundationHandler();
//        foundationHandler.setFNM(FNM);
//        foundationHandler.setTelemetry(telemetry);
        //init mode
        setup();
        //test();
        waitForStart();
        //false = up

        drive(2,51,0.9);
        drive(3,72,0.9);
        drive(3,5,0.15);
        foundationCatchRelease(true);
        drive(4,70,0.15);

        foundationCatchRelease(false);
        drive(1,86,0.9);
        drive(3,45,0.9);//55
        drive(2,15,0.9);
//        drive(3,8,0.9);//18//12
        drive(1,15,0.9);//37//28
        //drive(1,20,0.9);
        //drive(4,25,0.9);
        //drive(1,70,0.9);
        //drive(2,5,0.9);


        // drive(6,78);
        //foundationCatchRelease(true,increment);
        // sleep(1000);
        //  drive(4, 74);
        //  foundationCatchRelease(false,increment);
        // sleep(30000);


//        drive(1,30)       ;
        //int distance=distanceToTurn(90);
//        drive(6,distance)
        /*
        drive(2, 20);
        //drive bwd to be front the middle of the foundation
        drive(3, 78);
        //drive right to hook the foundation
        foundationCatchRelease(true,increment);
        sleep(5000);
        //should be a hook command to pick up the foundation
        drive(4, 74);
        //when play button is pressed
        sleep(5000);
        foundationCatchRelease(false,increment);
        //should be a hook command to drop down the foundation
        drive(1,78);
        //to push the foundation
        drive(3,31);
        //to push the foundation
        drive(2,36);
        //to push foundation
        drive(1, 50);
        //park da robo

*/
        while (opModeIsActive()) {
        }

    }

}
