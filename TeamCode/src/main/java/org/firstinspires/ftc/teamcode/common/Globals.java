package org.firstinspires.ftc.teamcode.common;

public class Globals {

    public static boolean TA_PONTUANDO = false;
    public static boolean TA_INTAKE = false;
    public static boolean TA_STORED = false;

    public static void startPontuar() {
        TA_PONTUANDO = true;
        TA_INTAKE = false;
    }

    public static void pararPontuar(){
        TA_PONTUANDO = false;
        TA_INTAKE = false;
    }

    public static void startIntaking() {
        TA_PONTUANDO = false;
        TA_INTAKE = true;
    }

    public static void pararIntaking() {
        TA_PONTUANDO = false;
        TA_INTAKE = false;
    }
}
