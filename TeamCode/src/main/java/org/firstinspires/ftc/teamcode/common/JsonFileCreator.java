package org.firstinspires.ftc.teamcode.common;
import android.content.Context;
import android.util.Log;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

public class JsonFileCreator {

    public static void createJsonFile(Context context) {
        // Criando um objeto JSON
        JSONObject jsonObject = new JSONObject();
        try {
            jsonObject.put("nome", "João");
            jsonObject.put("idade", 25);
            jsonObject.put("cidade", "São Paulo");

            JSONArray hobbiesArray = new JSONArray();
            hobbiesArray.put("Futebol");
            hobbiesArray.put("Leitura");
            hobbiesArray.put("Música");

            jsonObject.put("hobbies", hobbiesArray);

        } catch (JSONException e) {
            e.printStackTrace();
        }

        // Convertendo o objeto JSON para string
        String jsonString = jsonObject.toString();

        // Salvando o JSON em um arquivo local
        File file = new File(context.getFilesDir(), "dados.json");
        try (FileOutputStream fos = new FileOutputStream(file)) {
            fos.write(jsonString.getBytes());
            Log.d("JsonFileCreator", "Arquivo JSON criado com sucesso!");
        } catch (IOException e) {
            e.printStackTrace();
            Log.e("JsonFileCreator", "Erro ao criar o arquivo JSON!");
        }
    }
}
