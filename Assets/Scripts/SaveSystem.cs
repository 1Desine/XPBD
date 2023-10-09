using UnityEngine;
using System.IO;
using Newtonsoft.Json;

public static class SaveSystem {


    public static void WriteJson(string pathAndName, string json) {
        File.WriteAllText(Application.dataPath + pathAndName, json);
    }
    public static string ReadJson(string relativePath) {
        string json = File.Exists(Application.dataPath + relativePath) ? File.ReadAllText(Application.dataPath + relativePath) : null ;
        if (json == null) Debug.LogError("me. Did not found file at: " + relativePath);
        return json;
    }


    public static string SerializeJson(object obj) {
        return JsonConvert.SerializeObject(obj, Formatting.Indented,
            new JsonSerializerSettings {
                ReferenceLoopHandling = ReferenceLoopHandling.Ignore,
                NullValueHandling = NullValueHandling.Ignore,
            });
    }
    public static T DeserializeJson<T>(string json) {
        return JsonConvert.DeserializeObject<T>(json);
    }

}
