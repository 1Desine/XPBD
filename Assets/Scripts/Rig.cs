#if UNITY_EDITOR
using UnityEditor;
#endif
using System.Collections.Generic;
using UnityEngine;
using System;

public class Rig : MonoBehaviour {
    [SerializeField] private List<Holder> serDefHolder = new List<Holder>();
    [SerializeField] private List<Holder> serUndefHolder;
    private List<Holder> unserDefHolder = new List<Holder>();
    private List<Holder> unserUndefHolder;

    [SerializeField] private List<Point> serDefPoints = new List<Point>();
    [SerializeField] private List<Point> serUndefPoints;
    private List<Point> unserDefPoints = new List<Point>();
    private List<Point> unserUndefPoints;

    private void Awake() {
        /* does work, but not needed
        Debug.LogWarning("Serialized Defined <i>holder</i>");
        Debug.Log("Count: " + serDefHolder.Count);
        if (serDefHolder[0].p_R == serDefHolder[1].p_L) Pass(); else Fail();
        */

        if (serUndefHolder[0].p_R.id != null) Debug.Log("p_R info: " + serUndefHolder[0].p_R.id);

        Debug.LogWarning("Serialized Undefined <i>holder</i>");
        Debug.Log("Count: " + serUndefHolder.Count);
        if (serUndefHolder[0].p_R == serUndefHolder[1].p_L) Pass(); else Fail();

        /* Full data loss
        Debug.LogWarning("Unserialized Defined <i>holder</i>");
        Debug.Log("Count: " + unserDefHolder.Count);
        if (unserDefHolder[0].p_R == unserDefHolder[1].p_L) Pass(); else Fail();

        Debug.LogWarning("Unserialized Undefined <i>holder</i>");
        Debug.Log("Count: " + unserUndefHolder.Count);
        if (unserUndefHolder[0].p_R == unserUndefHolder[1].p_L) Pass(); else Fail();
        */
    }

    private void Pass() => Debug.Log("Result: <b><color=#00ff00ff>Pass</color></b>");
    private void Fail() => Debug.Log("Result: <b><color=#ff0000ff>Fail</color></b>");


    [Serializable]
    private class Point { public int? id; }

    [Serializable]
    private class Holder {
        public Point p_R;
        public Point p_L;
    }


#if UNITY_EDITOR
    [CustomEditor(typeof(Rig))]
    public class RigEditor : Editor {
        public override void OnInspectorGUI() {
            base.OnInspectorGUI();
            Rig rig = (Rig)target;

            if (GUILayout.Button("Reset")) {
                List<Point> pointsListToUse = rig.serDefPoints;
                pointsListToUse = new List<Point>();
                pointsListToUse.Add(new Point());

                rig.serDefHolder.Add(new Holder());
                rig.serDefHolder.Add(new Holder());
                rig.serDefHolder[0].p_R = pointsListToUse[0];
                rig.serDefHolder[1].p_L = pointsListToUse[0];


                rig.serUndefHolder = new List<Holder>();
                rig.serUndefHolder.Add(new Holder());
                rig.serUndefHolder.Add(new Holder());
                rig.serUndefHolder[0].p_R = pointsListToUse[0];
                rig.serUndefHolder[1].p_L = pointsListToUse[0];


                rig.unserDefHolder.Add(new Holder());
                rig.unserDefHolder.Add(new Holder());
                rig.unserDefHolder[0].p_R = pointsListToUse[0];
                rig.unserDefHolder[1].p_L = pointsListToUse[0];


                rig.unserUndefHolder = new List<Holder>();
                rig.unserUndefHolder.Add(new Holder());
                rig.unserUndefHolder.Add(new Holder());
                rig.unserUndefHolder[0].p_R = pointsListToUse[0];
                rig.unserUndefHolder[1].p_L = pointsListToUse[0];


                rig.serUndefHolder[0].p_R.id = 1;
            }
        }
    }
#endif


}
