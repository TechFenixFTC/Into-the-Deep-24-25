package org.firstinspires.ftc.teamcode.subsystems.common.Horizontal;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class LinearHorizontal {
    public Servo servoLinearHorizonal;
    public DcMotorEx motorLinearHorizontal;
    public int targetPositionLinearHorizontal = 0;
    public boolean teleop = false;
    public boolean precisaTerminarDeExtender = false, precisaTerminarDeRetrair = false;
    LinearHorizontalStates linearHorizontalStates = LinearHorizontalStates.RETRACTED;
    private double delayHoriontal = 0;
    public boolean intaking = false, modoAPENASTEMPO = false;

    public static double kp = 0.001, kd = 0.000, kff = 0.0000, kll = 0.0000;
    public LinearHorizontal(HardwareMap hardwareMap) {


        this.servoLinearHorizonal = hardwareMap.get(Servo.class, "porta4");
        this.motorLinearHorizontal = hardwareMap.get(DcMotorEx.class, "linearH");
        this.motorLinearHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorLinearHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        reset();

    }
    public void upSetPoint() {
        //if( this.targetPositionLinearHorizontal > 0.99) return;
        //this.targetPositionLinearHorizontal += 0.05;
        //this.servoLinearHorizonal.setPosition(targetPositionLinearHorizontal);
        //  if( this.targetPositionLinearHorizontal >  80) return;
        this.targetPositionLinearHorizontal += 140;
    }
    public void downSetPoint() {

        //this.targetPositionLinearHorizontal -= 0.05;
        //this.servoLinearHorizonal.setPosition(targetPositionLinearHorizontal);

        this.targetPositionLinearHorizontal -= 140;
    }
    public void reset() {
        targetPositionLinearHorizontal = 0;
        motorLinearHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLinearHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public Action recolher(double runTime, double delay) {

        if (delay > 0) {
            precisaTerminarDeRetrair = true;
            precisaTerminarDeExtender = false;
            delayHoriontal = runTime + delay;
            return new InstantAction(() -> {});
        }
        return new Action() {
            int margin;
            //Timer timer = new Timer();
            ElapsedTime time = new ElapsedTime();

            boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started) {
                    targetPositionLinearHorizontal = 0;
                    time.reset();
                    started = true;
                }


                margin  = 100;

                double positionError = controladorDePosicao();
                motorLinearHorizontal.setPower(-1);
                if ((time.milliseconds() > 1200 && Math.abs(motorLinearHorizontal.getVelocity()) < 5) || motorLinearHorizontal.getCurrentPosition() < 30) {
                    if(!teleop) reset();
                    return false;
                }
                return Math.abs(positionError) > margin;
            }

        };


    }
    public Action extender(double runTime, double delay) {
        if (delay > 0) {
            precisaTerminarDeExtender = true;
            precisaTerminarDeRetrair = false;
            delayHoriontal = runTime + delay;
            return new InstantAction(() -> {});
        }
        if(modoAPENASTEMPO) {
            return new Action() {
                int margin;
                ElapsedTime time = new ElapsedTime();
                boolean started = false;
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    if(!started) {

                        time.reset();
                        targetPositionLinearHorizontal = 3100;
                        started = true;
                        intaking = true;
                    }


                    margin  = 200;

                    double positionError = controladorDePosicao();
                    if(time.time() > 1){
                        intaking = false;
                        return false;
                    }
                    if(!(Math.abs(positionError) > margin)) {
                        intaking = false;
                    }
                    return Math.abs(positionError) > margin;
                }

            };
        }
        return new Action() {
            int margin;
            ElapsedTime time = new ElapsedTime();
            boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started) {

                    time.reset();
                    targetPositionLinearHorizontal = 3100;
                    started = true;
                    intaking = true;
                }


                margin  = 200;

                double positionError = controladorDePosicao();
                if((time.time() >  0.6 && Math.abs(motorLinearHorizontal.getVelocity()) < 15) || time.time() > 1){
                    intaking = false;
                    return false;
                }
                if(!(Math.abs(positionError) > margin)) {
                    intaking = false;
                }
                return Math.abs(positionError) > margin;
            }

        };


    }
    public Action extenderNemTanto(double runTime, double delay) {
        if (delay > 0) {
            precisaTerminarDeExtender = true;
            precisaTerminarDeRetrair = false;
            delayHoriontal = runTime + delay;
            return new InstantAction(() -> {});
        }
        return new Action() {
            int margin;
            ElapsedTime time = new ElapsedTime();
            boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started) {
                    time.reset();
                    targetPositionLinearHorizontal = 2200;
                    started = true;
                }


                margin  = 200;

                double positionError = controladorDePosicao();
                if((time.time() >  1.4 && Math.abs(motorLinearHorizontal.getVelocity()) < 15) || time.time() > 1){

                   return false;
                }
                return Math.abs(positionError) > margin;
            }

        };


    }

    public double controladorDePosicao() {

        // FeedForward

        double ff = targetPositionLinearHorizontal  * kff;

        //KP
        double p = kp;

        //Cria o Controlador PID
        PIDController controller = new PIDController(kp, 0, kd);
        controller.setPID(kp, 0, kd);



        //Calcular correção
        double pid = controller.calculate(this.motorLinearHorizontal.getCurrentPosition(), targetPositionLinearHorizontal);

        if((motorLinearHorizontal.getCurrentPosition() < 50 && targetPositionLinearHorizontal < 50)&& Math.abs(controller.getPositionError())<30) {
            this.motorLinearHorizontal.setPower(0);
            return controller.getPositionError();
        }else {
            this.motorLinearHorizontal.setPower(pid + ff);
            if(controller.getPositionError() < -220){
                this.motorLinearHorizontal.setPower(-1);
            }
        }



        return controller.getPositionError();

    }

    /*===========================*\
                GETTERS
     /*===========================*/

}
