package org.firstinspires.ftc.teamcode.common;

import android.os.Environment;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class LogJsonManager {

    private File logFile;
    private JSONArray logArray;

    public LogJsonManager(String filename) {
        try {
            // Define o diretório onde o log será salvo
            File directory = new File(Environment.getExternalStorageDirectory(), "FTCLogs");
            if (!directory.exists()) {
                directory.mkdirs(); // Cria a pasta se não existir
            }

            // Define o arquivo de log
            logFile = new File(directory, filename);
            if (!logFile.exists()) {
                logFile.createNewFile();
            }

            // Inicializa o array de logs
            logArray = new JSONArray();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // Método para adicionar um novo log
    public void addLog(String tag, String message) {
        try {
            JSONObject logEntry = new JSONObject();
            logEntry.put("timestamp", System.currentTimeMillis());
            logEntry.put("tag", tag);
            logEntry.put("message", message);

            logArray.put(logEntry);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    // Método para salvar o log no arquivo JSON
    public void saveLogs() {
        try (FileWriter writer = new FileWriter(logFile, false)) {
            writer.write(logArray.toString(4)); // Formata JSON com indentação
            writer.flush();
        } catch (IOException | JSONException e) {
            e.printStackTrace();
        }
    }
}
