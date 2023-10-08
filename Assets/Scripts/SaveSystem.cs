using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public static class SaveSystem {

    public static void WriteJson(string pathAndName, string json) {
        File.WriteAllText(Application.dataPath + pathAndName, json);
    }
    public static string ReadJson(string pathAndName) {
        if (File.Exists(Application.dataPath + pathAndName)) {
            return File.ReadAllText(Application.dataPath + pathAndName);
        }
        Debug.LogError("me. Could not find: " + Application.dataPath + pathAndName);
        return null;
    }



}
