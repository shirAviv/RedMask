package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

@TeleOp(name = "-- 17182 OPMode ver 4 -- ", group = "")
public class Robot_Driver extends LinearOpMode {


    DcMotor R0, R2, L1, L3, CE1, CE2, SCM;
    Servo FNM, SHELFMOTOR, CATCHMOTOR, CM;
    float dir = 1;
    float curr_power = (float) 0.65;
    private double shelf_counter = 0;
    private double shelf_increment = 0.0055;

    void goFwd(float power) {
        R0.setPower(power);
        R2.setPower(-power * (dir));
        L1.setPower(power);
        R2.setPower(-power * (dir));
    }

    void goBwd(float power) {
        R0.setPower(-power);
        R2.setPower(power * (dir));
        L1.setPower(-power);
        R2.setPower(power * (dir));

    }

    public void setMotionEnginesMotorPower(float power) {
        if (power>0.3) {
            //R2.setPower(power-0.3);
            R0.setPower(power-0.3);
        } else {
            //R2.setPower(power);
            R0.setPower(power);
        }
        R2.setPower(power);
        L1.setPower(power);
        L3.setPower(power);
    }

    public void setCEPower(float power) {
        CE1.setPower(power);
        CE2.setPower(power);
    }

    public void setSCMPower(float power) {
        SCM.setPower(power);
    }

    public void foundationCatchRelease(boolean down) {


        telemetry.addData("FNM position", FNM.getPosition());
        if (down) {
            FNM.setDirection(Servo.Direction.REVERSE);
            FNM.setPosition(0.1);
            //          counter = counter - increment;
        } else {
            FNM.setDirection(Servo.Direction.FORWARD);
            FNM.setPosition(0.02);
//            counter = counter + increment;
        }
        telemetry.addData("FNM position after", FNM.getPosition());
        telemetry.update();
        sleep(10);//used to be 3000


        FNM.setDirection(Servo.Direction.FORWARD);
    }

    public void catchReleaseMotor(boolean pushin) {


        telemetry.addData("CRM position", CATCHMOTOR.getPosition());
        if (pushin) {
            CATCHMOTOR.setPosition(0.1);
        } else {

            CATCHMOTOR.setPosition(0.8);
        }
        telemetry.addData("CRM position after", CATCHMOTOR.getPosition());
        telemetry.update();



    }
    public void drag(boolean lower) {

        telemetry.addData("CRM position", CATCHMOTOR.getPosition());
        if (lower) {
            CM.setDirection(Servo.Direction.REVERSE);
            CM.setPosition(0.55);
        } else {
            CM.setDirection(Servo.Direction.FORWARD);
            CM.setPosition(0.55);
        }


        CM.setDirection(Servo.Direction.FORWARD);
    }


    void stand() {
        R0.setPower(0); //fwd
        R2.setPower(0); //rev
        L1.setPower(0); //fwd
        R2.setPower(0); //rev
    }

    void printData() {

        telemetry.addData("Port conection: ", "");
        telemetry.addLine("------------------------");
        telemetry.addData("R0 : ", R0.getPortNumber());
        telemetry.addLine("------------------------");
        telemetry.addData("R2 : ", R2.getPortNumber());
        telemetry.addLine("------------------------");
        telemetry.addData("L1 : ", L1.getPortNumber());
        telemetry.addLine("------------------------");
        telemetry.addData("L3 : ", L3.getPortNumber());
        telemetry.addLine("------------------------");
        telemetry.update();
        sleep(2000);


    }

    public void setup() {

        R0 = hardwareMap.get(DcMotor.class, "R0");
        R2 = hardwareMap.get(DcMotor.class, "R2");
        L1 = hardwareMap.get(DcMotor.class, "L1");
        L3 = hardwareMap.get(DcMotor.class, "L3");
        CE1 = hardwareMap.get(DcMotor.class, "CE1");//cube eater 1 (right)
        CE2 = hardwareMap.get(DcMotor.class, "CE2");//cube eater 2 (left)
        SCM = hardwareMap.get(DcMotor.class, "SCM"); //seazer motor (misparaim)
        FNM = hardwareMap.get(Servo.class, "FNM");
        CM = hardwareMap.get(Servo.class, "CM");
        CATCHMOTOR = hardwareMap.get(Servo.class, "CATCHMOTOR");
        SHELFMOTOR = hardwareMap.get(Servo.class, "SHELFMOTOR");
        R0.setDirection(DcMotor.Direction.FORWARD);
        R2.setDirection(DcMotor.Direction.FORWARD);
        L1.setDirection(DcMotor.Direction.FORWARD);
        L3.setDirection(DcMotor.Direction.FORWARD);
        CE1.setDirection(DcMotor.Direction.FORWARD);
        CE2.setDirection(DcMotor.Direction.FORWARD);
        SCM.setDirection(DcMotor.Direction.FORWARD);


        telemetry.addData("Stauts", "success!");
        telemetry.update();
        printData();

        waitForStart();

    }

    @Override
    public void runOpMode() throws InterruptedException {

        setup();


        while (opModeIsActive()) {

            //right
            if (gamepad1.left_stick_x > 0.3 || gamepad1.dpad_right) {
                telemetry.addData("Stauts", "right!");
                telemetry.update();
                R0.setDirection(DcMotor.Direction.FORWARD);
                R2.setDirection(DcMotor.Direction.REVERSE);
                L1.setDirection(DcMotor.Direction.REVERSE);
                L3.setDirection(DcMotor.Direction.FORWARD);
                setMotionEnginesMotorPower(curr_power);
                //left
            } else if (gamepad1.left_stick_x < -0.3 || gamepad1.dpad_left) {
                telemetry.addData("Stauts", "left!");
                telemetry.update();
                R0.setDirection(DcMotor.Direction.REVERSE);
                R2.setDirection(DcMotor.Direction.FORWARD);
                L1.setDirection(DcMotor.Direction.FORWARD);
                L3.setDirection(DcMotor.Direction.REVERSE);
                setMotionEnginesMotorPower(curr_power);
                //forward
            } else if (gamepad1.left_stick_y > 0.3 || gamepad1.dpad_down) {
                telemetry.addData("Stauts", "backward!");
                telemetry.update();
                R0.setDirection(DcMotor.Direction.FORWARD);
                R2.setDirection(DcMotor.Direction.REVERSE);
                L1.setDirection(DcMotor.Direction.FORWARD);
                L3.setDirection(DcMotor.Direction.REVERSE);
                setMotionEnginesMotorPower(curr_power);
                //backward
            } else if (gamepad1.left_stick_y < -0.3 || gamepad1.dpad_up) {
                telemetry.addData("Stauts", "forward!");
                telemetry.update();
                R0.setDirection(DcMotor.Direction.REVERSE);
                R2.setDirection(DcMotor.Direction.FORWARD);
                L1.setDirection(DcMotor.Direction.REVERSE);
                L3.setDirection(DcMotor.Direction.FORWARD);
                setMotionEnginesMotorPower(curr_power);
            }

            //left rotate
            else if (gamepad1.b) {
                R0.setDirection(DcMotor.Direction.FORWARD);
                R2.setDirection(DcMotor.Direction.FORWARD);
                L1.setDirection(DcMotor.Direction.FORWARD);
                L3.setDirection(DcMotor.Direction.FORWARD);
                setMotionEnginesMotorPower(curr_power);
            }//right rotate
            else if (gamepad1.x) {
                R0.setDirection(DcMotor.Direction.REVERSE);
                R2.setDirection(DcMotor.Direction.REVERSE);
                L1.setDirection(DcMotor.Direction.REVERSE);
                L3.setDirection(DcMotor.Direction.REVERSE);
                setMotionEnginesMotorPower(curr_power);
            } else if (gamepad1.right_bumper) {
                //normal mode
                curr_power = (float) 0.65;

            } else if (gamepad1.left_bumper) {
                //slow mode
                curr_power = (float) 0.40;
            }
            //pause
            else if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) {
                telemetry.addData("Stauts", "brake!");
                telemetry.update();
                setMotionEnginesMotorPower((float) 0);
                R0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                R2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                L3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                sleep(10);
            }
            if (gamepad1.a) {
                telemetry.addData("working on FNM 1", FNM.getPosition());
                telemetry.update();
                foundationCatchRelease(true);

            } else if (gamepad1.y) {
                telemetry.addData("working on FNM 2", FNM.getPosition());
                telemetry.update();
                foundationCatchRelease(false);
            }
            if(gamepad2.dpad_down){
            drag(true);



            }
            if (gamepad2.dpad_up){
                drag(false);
            }
            if (gamepad2.right_bumper) {
                telemetry.addData("Stauts", "cube_catch!");
                telemetry.update();
                //bring cube inside
                CE1.setDirection((DcMotorSimple.Direction.FORWARD));
                CE2.setDirection((DcMotorSimple.Direction.REVERSE));
                setCEPower((float) 0.7);

            } else if (gamepad2.left_bumper) {
                //take cube outside
                telemetry.addData("Stauts", "cube_out!");
                telemetry.update();
                CE1.setDirection((DcMotorSimple.Direction.REVERSE));
                CE2.setDirection((DcMotorSimple.Direction.FORWARD));
                setCEPower((float) 0.7);
            } else {
                setCEPower((float) 0);
                CE1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                CE2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                sleep(10);

            }


            // if (gamepad2.right_trigger > 0.2 ) {
            //   SCM.setDirection((DcMotorSimple.Direction.FORWARD));
            // setSCMPower((float) 0.7);

            // }
            if (gamepad2.b) {

                catchReleaseMotor(true);
            }
            if (gamepad2.x) {

                catchReleaseMotor(false);
            }

            while ( gamepad2.a && opModeIsActive()) {
                telemetry.addData("in right servo1 before move counter=", SHELFMOTOR.getPosition());
                telemetry.update();
                sleep(10);

                if (shelf_counter <= 1){
                    shelf_counter = shelf_counter + shelf_increment;
                    SHELFMOTOR.setPosition(shelf_counter);
                }

                //if (!gamepad2.a) {
                  //  break;
                //}
                telemetry.addData("in right servo1 counter=", SHELFMOTOR.getPosition());
                telemetry.update();

            }
            while (gamepad2.y && opModeIsActive()) {
                telemetry.addData("in left servo1 before move counter=", SHELFMOTOR.getPosition());
                telemetry.update();

                sleep(10);
                if (shelf_counter >= 0){
                    shelf_counter = shelf_counter - shelf_increment;
                    SHELFMOTOR.setPosition(shelf_counter);
                }
                //if (!gamepad2.y) {
                  //  break;
                //}
                telemetry.addData("in left servo1 counter=", SHELFMOTOR.getPosition());
                telemetry.update();
            }
            if (!gamepad2.y && !gamepad2.a) {
                SHELFMOTOR.setPosition(SHELFMOTOR.getPosition());
            }



//            else {
//                telemetry.addData("left t", gamepad2.left_trigger);
//                telemetry.update();
            if (gamepad2.right_trigger > 0.2) {
                sleep(10);
                SCM.setDirection((DcMotorSimple.Direction.FORWARD));
                setSCMPower((float) 1);
            } else {
                if (gamepad2.left_trigger > 0.2) {
                    SCM.setDirection((DcMotorSimple.Direction.REVERSE));
                    setSCMPower((float) 1);
//                    telemetry.addData("status", "scissors stopped!")
                } else {
                    setSCMPower((float) 0.0);

                }
            }
//            }


        }
    }
}



