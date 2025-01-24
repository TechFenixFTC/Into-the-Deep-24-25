package org.firstinspires.ftc.teamcode.agregadoras.agregadorasSubsistemas.Superior;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarra.BracoGarraV4;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Garra.GarraSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Horizontal.LinearHorizontalSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.LinearVertical.LinearVertical;

public class SubsistemasSuperiores {

        public Telemetry telemetry;
        public LinearVertical linearVertical;
        public LinearHorizontalSuperior horizontalSuperior;

        public GarraSuperior garraSuperior;
        public BracoGarraV4 braco;
        //public List<Action> runningActions2 = new ArrayList<>();
        public OrdersManager carteiro = new OrdersManager(telemetry);
        HardwareMap hardwaremap;





        public SubsistemasSuperiores(HardwareMap hardwareMap, Telemetry telemetry) {

            this.hardwaremap = hardwareMap;
            this.telemetry = telemetry;
            this.braco = new BracoGarraV4(hardwareMap,telemetry);
            this.linearVertical = new LinearVertical(hardwareMap);
            this.horizontalSuperior = new LinearHorizontalSuperior(hardwareMap);
            this.garraSuperior = new GarraSuperior(hardwareMap);
            //this.runningActions2 = new ArrayList<>();



            // todo: Fazer uma agregadora separada para os subsistemas inferiores - By miguel
        }


        private double getVoltage() {return hardwaremap.voltageSensor.iterator().next().getVoltage();}

    }


