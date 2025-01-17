package org.firstinspires.ftc.teamcode.subsystems.Vertex;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class LinearVertical {
    public final DcMotorEx motorR;
    public final DcMotorEx motorL;
    public final DigitalChannel led;

    // controla descer
    public boolean needToChangeTarget = false;
    public double timeToChangeTarget = 0;
    public int wantedTarget = 0;


    public int position;
    public double power;

    public int targetPosition = 0;
    public static double p = 0.008, i = 0, d = 0.008,f = 0.000;

    /* POSIÇÕES PRESETS */
    private int lowBasketPos = 3500, drivingPos = 200, intakingPos = 10;


    public LinearVertical (HardwareMap hardwareMap) {
        this.motorL =  hardwareMap.get(DcMotorEx.class, "linearL");
        this.motorR =  hardwareMap.get(DcMotorEx.class, "linearR");
        this.led =  hardwareMap.get(DigitalChannel.class, "1ledR");//atualizar no drive hub
        this.power = motorR.getPower();

        this.motorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorR.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorL.setDirection(DcMotorSimple.Direction.FORWARD);
        this.reset();
        this.position = motorR.getCurrentPosition();
        this.targetPosition = this.position;
    }

    public void reset() {
        this.motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double PIDF(boolean auto) {

        double kp = p;
        // FeedForward
        if (this.motorR.getCurrentPosition() < 1200) {
            f = f/2;
        }
        double ff = targetPosition  * f;
        //KP
        if (this.targetPosition < 500 && (this.motorR.getCurrentPosition() < 1000)  ) {
            kp = p/2;
        }
        if (this.targetPosition < 500 && (this.motorR.getCurrentPosition() > 1000)  ) {
            kp = p/3;
        }
        //Cria o Controlador PID
        PIDController controller = new PIDController(kp, i, d);
        controller.setPID(kp, i, d);


        //Calcular correção
        double pid = controller.calculate(this.motorR.getCurrentPosition(), targetPosition);

        // turn off the motors if the linear is between 600 and 1150 ticks and error is low
        if(Math.abs(controller.getPositionError()) < 30 && (this.motorR.getCurrentPosition() < 200) && (this.motorR.getCurrentPosition() < 200) ) {
            motorR.setPower(0);
            motorL.setPower(0);
            return controller.getPositionError();
        }
        if(this.motorR.getCurrentPosition() < 270 && targetPosition < 10 && targetPosition > -30) {
            motorR.setPower(0);
            motorL.setPower(0);
            return controller.getPositionError();
        }

        if(targetPosition > 3000 && motorR.getCurrentPosition() < 3000) pid = 1;
        motorR.setPower(pid+ff );//COLOCAR O ff depois
        motorL.setPower(pid+ff);


        /*===CODIGO DE ERRO===*/
        //zera os motores e lida o led


        return controller.getPositionError();

        }
    public void handleElevadorTeleop(Telemetry telemetry, double runTime) {
        if (  runTime >= timeToChangeTarget && needToChangeTarget) {
            targetPosition = wantedTarget;
            needToChangeTarget = false;
        }

        PIDF(false);

    }
    public void upSetPoint() {
        this.targetPosition += 80;
    }
    public void downSetPoint() {
        this.targetPosition -= 80;
    }
    public void changeTarget(double runTime, double delay, int target) {
        timeToChangeTarget = runTime + delay;
        needToChangeTarget = true;
        wantedTarget = target;
    }

    public Action ElevadorGoTo(int target) {

        return new Action() {
            int margin;
            ElapsedTime time = new ElapsedTime();
            boolean started = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started) {
                    time.reset();
                    started = true;
                }
                targetPosition = target;
                if (target > 2500) {
                    margin = 300;
                }
                else {
                    margin  = 200;
                }
                if (targetPosition < 100 ) {
                    margin  = 30;
                }


                double positionError = 0;
                if(targetPosition > 0 ) {
                    positionError = PIDF(true);
                }
                else {
                    positionError = targetPosition - motorR.getCurrentPosition();
                    motorL.setPower(-1);
                    motorR.setPower(-1);
                }
                if (time.milliseconds() > 800 && Math.abs(motorL.getVelocity()) < 15 && targetPosition < 100 ) {
                    reset();
                    return false;
                }
                if(targetPosition < 700 && Math.abs(positionError) < margin){
                    motorL.setPower(0);
                    motorR.setPower(0);
                }
                return Math.abs(positionError) > margin;
            }

        };

    }

}
