package org.firstinspires.ftc.teamcode.subsystems.common.Horizontal;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.HashMap;

@Config
public class LinearHorizontal {
    protected Servo servoLinearHorizontal;
    public ElapsedTime tempoIndoAteOsetPoint = new ElapsedTime();
    public DcMotorEx motorHorizontal;
    PIDController controller = new PIDController(p, i, d);
    public static boolean monitor;
    public static double p = 0.0018, i = 0, d = 0.000,f = 0, ll = 0, kll = 0;
    private  int margem = 10, margemAut = 20 , sense = 4;
    public static int targetPosition = 0;
    public LinearHorizontalStates linearHorizontalInferiorState = LinearHorizontalStates.RETRACTED;
    public LinearHorizontal(HardwareMap hardwareMap, String hardwareName) {
        this.servoLinearHorizontal = hardwareMap.get(Servo.class,"porta5c" );
        this.motorHorizontal = hardwareMap.get(DcMotorEx.class,hardwareName);
        this.motorHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public double PID() {

        double kp = p;
        int linearpos = motorHorizontal.getCurrentPosition();

        controller.setPID(kp, i,d);
        double pid = controller.calculate(linearpos, targetPosition);

        motorHorizontal.setPower(pid);
        return pid ;
    }//todo não testado
    public Action horizontalToGo(int alvo){
        return new Action() {
            ElapsedTime time = new ElapsedTime();
            boolean started = false,condicaoDeParada = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started) {
                    targetPosition = alvo;
                    time.reset();
                    started = true;
                }
                PID();
                if(condicaoDeParada){
                    return false;
                }

                return true;
            }
        };
    }//todo não testado


    public Action goTo(int target){
        linearHorizontalInferiorState = LinearHorizontalStates.EXTENDED;
        return new Action() {
            ElapsedTime time = new ElapsedTime();
            private boolean start= false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!start) {
                    time.reset();
                }
                if(10< motorHorizontal.getCurrentPosition()&&motorHorizontal.getCurrentPosition()<120){
                    motorHorizontal.setPower(1);
                }
                if ( motorHorizontal.getCurrentPosition()>target && !start) {
                    motorHorizontal.setPower(0.3);
                    return false;
                }
                if(motorHorizontal.getCurrentPosition()<20&& !start){
                    motorHorizontal.setPower(-0.2);
                    return false;
                }

                return true;
            }
        };}


    public Action goToExtended(){
        return goTo(120);
    }
    public Action goToRetracted(){
        return goTo(0);
    }
    public void upSetPoint() {
        changeTarget(targetPosition + sense);
    }
    public void downSetPoint() {
        changeTarget(targetPosition - sense);
    }
    public void changeTarget(int target) {

        targetPosition = target;
        tempoIndoAteOsetPoint.reset();
    }

    public void reset(){
        this.motorHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void monitor(Telemetry telemetry,String hozizontal) {//todo okey
        if (monitor) {
            telemetry.addLine("======================================");
            telemetry.addLine("TELEMETRIA DO LINEAR HORIZONTAL ");
            telemetry.addLine("======================================");
            telemetry.addData("-Posição do Motor: ",motorHorizontal.getCurrentPosition());
            telemetry.addData("-Posição alvo: ",targetPosition);
            telemetry.addData("setPower1",motorHorizontal.getPower());
            //telemetry.addData("-PID ", PID());
            telemetry.addData("setPower2",motorHorizontal.getPower());
            telemetry.addData("porta", motorHorizontal.getPortNumber());
            telemetry.addData("corrente",motorHorizontal.getCurrent(CurrentUnit.AMPS));
            //telemetry.addData("modo",motorHorizontal.getMode());

        }
    }




}