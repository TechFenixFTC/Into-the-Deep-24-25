package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class HardwareNames {
    public static String

    /**************************************************
    *                  Rodas Mortas                   *
    **************************************************/
        par0 = "",
        par1 = "",
        perp = "",
    /**************************************************
     *                  Motores Chassi                *
     **************************************************/
        rightFront =  "rightFront",
        leftFront =   "leftFront",
        rightBack =   "rightBack",
        leftBack =    "leftBack",
    /**************************************************
    *                  Sensores                       *
    **************************************************/
        distanceSensorL      = "sensorporta3",
        distanceSensorR      = "sensorPorta2",
        colorSensor1         = "color1",
        colorSensor2         = "sensorPortaE2",

    /**************************************************
    *         Visão Computacional / Cameras          *
    **************************************************/
        webcam1              =  "Webcam 1",
        limelight            =  "limelight",
    /**************************************************
    *              DcMotors Subsistemas               *
    **************************************************/   /* Motores Vertex */
        verticalR            = "verticalr",
        verticalL            = "verticall",
        bracoGarraSuperior   = "braco",
    /**************************************************
     *              Servo Subsistemas               *
     **************************************************/   /* Motores Vertex */

    horizontalSuperiorServo = "horizontalSuperior",
    horizontalInferiorServo = "horizontalInferior",
    bracoGarraSuperiorServo   = "bracoServo",
    /**************************************************
     *                 Servo-motores                  *
     **************************************************/
        servoRatacaoGarra    = "porta0",
        servoAberturaGarrra  = "porta1",
    /**************************************************
     *                 TroughBore                  *
     **************************************************/
    horizontalSuperior    = "porta0",
    getHorizontalInferior  = "porta1"
        ;

        public static String[] servoNames = {servoRatacaoGarra, servoAberturaGarrra};
}
