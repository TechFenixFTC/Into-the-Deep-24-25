package org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.SensorCor;

import java.util.HashMap;
@Config
public class IntakeSuccao{
    private V5 robot;
    public SensorCor colorSensorSugar;
    public int  red,
                blue,
                green,
                alpha;
    public Servo angulacao;
    public static boolean monitor;
    private double delay = 0.25;
    private double cooldownAberturaGarra =0;
    public static double power_Sugador = 0.7;
    public DcMotorEx sugador;
    public Servo alcapao;

    private HashMap<SugarAngulationStates , Double> mapAngulation = new HashMap<>();
    public  SugarAngulationStates sugarAngulationStates  = SugarAngulationStates.INITIAL;

    private HashMap<AlcapaoStates , Double> mapAlcapao = new HashMap<>();
    private AlcapaoStates alcapaoStates  = AlcapaoStates.TRASNFER;


    public IntakeSuccao(HardwareMap hardwareMap){
        angulacao = hardwareMap.get(Servo.class, HardwareNames.angulacaoSugarServo);
        sugador = hardwareMap.get(DcMotorEx.class,HardwareNames.SugadorMotorInferior);
        alcapao = hardwareMap.get(Servo.class,HardwareNames.alcapaoSugarServo);
        colorSensorSugar = new SensorCor(hardwareMap);

        mapAngulation.put(SugarAngulationStates.INTAKE, 0.086);

        mapAngulation.put(SugarAngulationStates.TRANSFER, 0.334);
        mapAngulation.put(SugarAngulationStates.INITIAL, 0.334);

        mapAngulation.put(SugarAngulationStates.READY_TOINTAKE, 0.300);


        mapAlcapao.put(AlcapaoStates.TOTALOPEN, 0.0);
        mapAlcapao.put(AlcapaoStates.INTAKE,0.616);
        mapAlcapao.put(AlcapaoStates.READY_TOINTAKE,1.0);
        mapAlcapao.put(AlcapaoStates.INITIAL,0.834);
        mapAlcapao.put(AlcapaoStates.INTAKE_SPECIMEN,0.582);
        mapAlcapao.put(AlcapaoStates.TRASNFER,0.672);




        // alcapão aberto 0.463
        // transfer 0.769
        // intake 0.674
    }
    public Action verifyColorSensor(){
        return new Action() {
            V5 robot;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if(colorSensorSugar.getAlpha() > 1500){
                    sugador.setPower(-1);
                    return false;
                }
                return true;
            }
        };
    }
    public Action GotoReadyToIntakeSpecimen(){
        return new InstantAction(()->{
            sugarAngulationStates  = SugarAngulationStates.READY_TOINTAKE;
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));

        });
    }

    public Action GotoIntakeSpecimen(){
        return new InstantAction(()->{
            sugarAngulationStates  = SugarAngulationStates.INTAKE;
            alcapaoStates = AlcapaoStates.READY_TOINTAKE;
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));
            alcapao.setPosition(mapAlcapao.get(alcapaoStates));

        });
    }
    public Action GotoReadyToIntakeSample(){
        return new InstantAction(()->{
            sugarAngulationStates  = SugarAngulationStates.READY_TOINTAKE;
            alcapaoStates = AlcapaoStates.READY_TOINTAKE;
            alcapao.setPosition(mapAlcapao.get(alcapaoStates));
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));


        });
    }

    public Action GotoIntakeSample(){
        return new InstantAction(()->{
            sugarAngulationStates  = SugarAngulationStates.INTAKE;
            alcapaoStates = AlcapaoStates.INTAKE;
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));
            alcapao.setPosition(mapAlcapao.get(alcapaoStates));

        });
    }

    public Action IntakeSugar(){
        return new InstantAction(()->{
            sugador.setPower(power_Sugador);
        });
    }
    public Action IntakeSugarMedio(){
        return new InstantAction(()->{
            sugador.setPower(0.5);
        });
    }
    public Action IntakeRepelir(){
        return new InstantAction(()->{
            sugador.setPower(power_Sugador * -1);
        });
    }
    public Action IntakeParar(){
        return new InstantAction(()->{
            sugador.setPower(0);
        });
    }
    public Action GoToFinishIntake(){
        return new InstantAction(()->{
            sugarAngulationStates = SugarAngulationStates.INITIAL;
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));


        });
    }
    public Action GoToExpulse(){
        return new InstantAction(()->{
            sugarAngulationStates = SugarAngulationStates.INITIAL;
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));


        });
    }
    public Action IntakePositionAlcapao(){
        return new InstantAction(()->{
            alcapaoStates = AlcapaoStates.INTAKE;
            alcapao.setPosition(mapAlcapao.get(alcapaoStates));
        });
    }
    public Action TransferPositionAlcapao(){
        return new InstantAction(()->{
            alcapaoStates = AlcapaoStates.TRASNFER;
            alcapao.setPosition(mapAlcapao.get(alcapaoStates));
        });
    }
    public Action ReadytoIntakePositionAlcapao(){
        return new InstantAction(()->{
            alcapaoStates = AlcapaoStates.READY_TOINTAKE;
            alcapao.setPosition(mapAlcapao.get(alcapaoStates));
        });
    }
    public Action TotalOpenPositionAlcapao(){
        return new InstantAction(() -> {
            alcapaoStates = AlcapaoStates.TOTALOPEN;
            alcapao.setPosition(mapAlcapao.get(alcapaoStates));
        });
    }
    public Action GoToInitial(){
        return new InstantAction(()->{
            sugarAngulationStates = SugarAngulationStates.INITIAL;
            alcapaoStates = AlcapaoStates.TRASNFER;
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));
            alcapao.setPosition(mapAlcapao.get(alcapaoStates));

        });
    }
    public Action GoToTransfer(){
        return new InstantAction(()->{

            alcapaoStates = AlcapaoStates.TRASNFER;
            alcapao.setPosition(mapAlcapao.get(AlcapaoStates.TRASNFER));

        });
    }
    public Action gerenciadorDoFechamentoDaAlcapaoNoTeleop(double runTime) {
        if(runTime < this.cooldownAberturaGarra) {
            return new InstantAction(() -> {});
        }
        this.cooldownAberturaGarra = runTime + this.delay;

        if (this.alcapaoStates == AlcapaoStates.INTAKE) {
            return this.TransferPositionAlcapao();
        }
        return this.IntakePositionAlcapao();

    }
    public Action pwmDiseable(){
        return  new InstantAction(()->{
            angulacao.getController().pwmDisable();
        });
    }
    public void monitor(Telemetry telemetry) {
        if (monitor) {
            telemetry.addLine("======================================");
            telemetry.addLine("       TELEMETRIA DO INTAKE SUCÇÃO    ");
            telemetry.addLine("======================================");
            telemetry.addData("getPosition angular",angulacao.getPosition());
            telemetry.addData("PMW status angular", angulacao.getController().getPwmStatus());
            telemetry.addData("getPoition alcapao", alcapao.getPosition());
            telemetry.addData("PMW status alcapao",alcapao.getController().getPwmStatus());
            telemetry.addData("Estado Atual da angulação do intake",sugarAngulationStates);
            telemetry.addData("corrente motor", sugador.getCurrent(CurrentUnit.AMPS));
            telemetry.addLine("===========================================");
            telemetry.addLine("       TELEMETRIA DO INTAKE COLORSENSOR    ");
            telemetry.addLine("===========================================");
            telemetry.addData("Alpha", colorSensorSugar.getAlpha());
            telemetry.addData("Red", colorSensorSugar.getRed());
            telemetry.addData("Blue",colorSensorSugar.getBlue());
            telemetry.addData("Green", colorSensorSugar.getGreen());
            telemetry.addData("Distancia", colorSensorSugar.getDistance());





            //telemetry.addData("",);

        }}
}
