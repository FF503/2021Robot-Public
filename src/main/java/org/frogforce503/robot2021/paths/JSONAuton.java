package org.frogforce503.robot2021.paths;

import edu.wpi.first.wpilibj.Filesystem;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.FileReader;
import java.io.IOException;

public class JSONAuton {
    String autonName, fileName;
    private JSONObject object;

    public JSONAuton(String autonName) {
        this.autonName = autonName;
        fileName = autonName;
        fileName += (!autonName.endsWith(".json")) ? ".json" : "";
    }

    public JSONPathNew getPath(String pathName) {
        JSONParser helper = new JSONParser();

        try {
            object = (JSONObject) helper.parse(new FileReader(Filesystem.getDeployDirectory().getAbsolutePath() + "/AutonJSONs/" + fileName));
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }

        JSONArray listOfPaths = (JSONArray) object.get("paths");

        for (int i = 0; i < listOfPaths.size(); i++) {
            JSONObject obj = (JSONObject) ((JSONArray) object.get("paths")).get(i);

            System.out.println(obj.get("name"));
            System.out.println(pathName);
            System.out.println("-------");
            if (obj.get("name").equals(pathName)) {
                return new JSONPathNew(obj);
            }
        }
        System.err.print("Learn to spell, you fool");
        return null;

        // return new JSONPathNew((JSONObject) ((JSONArray) object.get("paths")).get(0));


    }


}