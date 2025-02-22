package org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.SensorToque;

@Config
public class LinearHorizontalMotor {
    public ElapsedTime tempoIndoAteOsetPoint = new ElapsedTime();
    public DcMotorEx motorHorizontal;
    public SensorToque sensorToqueHorizontal;
    PIDController controller = new PIDController(p, i, d);
    public static boolean monitor, needToHold = false;
    private boolean isBusy;
    public static double p = 0.015, i = 0, d = 0.000,f = 0, ll = 0, kll = 0;
    public int position;
    private  int margem = 5, margemAut = 20 , sense = 4;
    public static int targetPosition = 0;
    public LinearHorizontalStates linearHorizontalInferiorState = LinearHorizontalStates.RETRACTED;
    public LinearHorizontalMotor(HardwareMap hardwareMap) {
        this.motorHorizontal = hardwareMap.get(DcMotorEx.class, HardwareNames.horizontalInferiorMotor);
        sensorToqueHorizontal = new SensorToque(hardwareMap);
        this.motorHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        this.position = motorHorizontal.getCurrentPosition();
        this.targetPosition = this.position;
        reset();

    }
    public double PID() {

        double kp = p;
        int linearpos = motorHorizontal.getCurrentPosition();

        controller.setPID(kp, i,d);
        double pid = controller.calculate(linearpos, targetPosition);

        motorHorizontal.setPower(pid);
        return pid ;
    }//todo não testado


    public Action turnOnHold() {
        return new InstantAction(() -> needToHold = true);
    }

    public Action turnOffHold() {
        return new InstantAction(() -> needToHold = false);
    }
    public Action hold() {
        return new Action() {
            ElapsedTime time = new ElapsedTime();
            boolean started = false,condicaoDeParada = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started) {
                    time.reset();
                    started = true;
                }
                PID();

                condicaoDeParada = needToHold;

                if(condicaoDeParada){
                    return false;
                }

                return true;
            }
        };
    }//todo não testado
    public Action horizontalGoTo(int alvo){
        return new Action() {
            ElapsedTime time = new ElapsedTime();
            boolean started = false,condicaoDeParada = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started) {
                    targetPosition = alvo;
                    time.reset();
                    started = true;
                    isBusy = false;
                }
                PID();
                                    // 125   >= 122 && 125 <= 142

                condicaoDeParada = motorHorizontal.getCurrentPosition() >= targetPosition - margem && motorHorizontal.getCurrentPosition() <= targetPosition + margem ;

                if(condicaoDeParada){
                    isBusy = false;
                    if(targetPosition < 10) {
                        linearHorizontalInferiorState = LinearHorizontalStates.RETRACTED;
                    }
                    if(targetPosition > 100) {
                        linearHorizontalInferiorState = LinearHorizontalStates.EXTENDED;
                    }
                    return false;

                }

                return true;
            }
        };
    }//todo não testado


    public Action goTo(int target){

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

        return horizontalGoTo(130);
    }
    public Action goToRetracted(){

        return horizontalGoTo(-15);
    }
    public void upSetPoint() {
        changeTarget(targetPosition + sense);
    }
    public void downSetPoint() {
        changeTarget(targetPosition - sense);
    }
    public void changeTarget(int target) {

        targetPosition = Range.clip(target,-100,160);
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
            telemetry.addData("Sensor de Toque isPressed", sensorToqueHorizontal.isPressed());
            // 0 não está tocando
            // 1 está tocando
            //telemetry.addData("-PID ", PID());
            telemetry.addData("setPower2",motorHorizontal.getPower());
            telemetry.addData("porta", motorHorizontal.getPortNumber());
            telemetry.addData("corrente",motorHorizontal.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Estado Atual", linearHorizontalInferiorState);
            telemetry.addData("Isbusy",isBusy);

        }
    }




}