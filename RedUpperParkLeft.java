package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "RedUpperParkLeft", group = "Linear Opmode")
public class RedUpperParkLeft extends LinearOpMode {

    DcMotor R0, R2, L1, L3, CE1, CE2, SCM;
    Servo FNM;
    BNO055IMU imu;
    Orientation angle;



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

        //set gyro setting
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode=BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!imu.isGyroCalibrated()) {
            telemetry.addData("calibrating imu","...");
            telemetry.update();
            sleep(50);
            idle();
        }

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

    public void fix_drive_angle() {
        sleep(1000);
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double curr_angle=angle.firstAngle;
        if (curr_angle<0) {
            curr_angle=curr_angle*-1;
        }
        double curr_pos_power=power+(curr_angle / 10);
        if (curr_pos_power>=1) {
            curr_pos_power=1;
        }
        double curr_minus_power=power-((curr_angle / 10));
        if (curr_minus_power<=0) {
            curr_minus_power=0.2;
        }
        if(angle.firstAngle > 0){

            R0.setPower(curr_minus_power);
            R2.setPower(curr_pos_power);
            L3.setPower(curr_pos_power);
            L1.setPower(curr_minus_power);
        }
        if (angle.firstAngle < 0){
            R0.setPower(curr_pos_power);
            R2.setPower(curr_minus_power);
            L3.setPower(curr_minus_power);
            L1.setPower(curr_pos_power);
        }
        telemetry.addData("angle",angle.firstAngle);
        telemetry.update();
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
            case 2: {
                L3.setDirection(DcMotor.Direction.REVERSE);
                L1.setDirection(DcMotor.Direction.FORWARD);
                R2.setDirection(DcMotor.Direction.REVERSE);
                R0.setDirection(DcMotor.Direction.FORWARD);
                break;
            }
            case 1 : {
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

        if (dir==1) {
            if (power>0.3) {
                R0.setPower(power-0.3);
            } else {
                R0.setPower(power);
            }
        }
        else {
            R0.setPower(power);
        }

        //calc the error - by ticks
        target_position = (L3.getCurrentPosition() + target) - error;
        // be safe with abs value
        //  error2 = Math.abs(target_position - 1328);
        // target_position = Math.abs(target_position - error2);


        //run to position
        while ((L3.getCurrentPosition() < target_position)) {

//            if (dir==1) {
//                fix_drive_angle();
//            }
//            else {
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
            if (dir==1) {
                if (power>0.3) {
                    R0.setPower(power-0.3);
                } else {
                    R0.setPower(power);
                }
            }
            else {
                R0.setPower(power);
            }
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
            FNM.setPosition(0.5);
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


        drive(2,50,0.9);
        drive(3,69,0.9);
        drive(3,5,0.15);
        foundationCatchRelease(true);
        drive(4,72,0.15);
        foundationCatchRelease(false);
        drive(1,85,0.9);//55
        drive(3,50,0.9);
        drive(2,30,0.9);

        drive(1,70,0.9);



        while (opModeIsActive()) {
        }

    }

}
