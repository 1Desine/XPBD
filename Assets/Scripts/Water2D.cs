/*
#if UNITY_EDITOR
using UnityEditor;
#endif
*/
using System;
using System.Collections.Generic;
using UnityEngine;

public class Water2D : MonoBehaviour {


    public int[] particlesList;


    [Serializable]
    private class Particle {
        public Vector2 position;
        public Vector2 previousPosition;
        public Vector2 velocity;
    }


    /*

    //
    // INSPECTOR
    //
    [ExecuteInEditMode]
    private void OnDrawGizmos() {
        Gizmos.color = Color.green;

        if (particlesList != null)
            foreach (Particle particle in particlesList) {
                Gizmos.color = Color.blue;
                Gizmos.DrawSphere(particle.position, 0.05f);
            }



        //// Bounds
        //if (linearConstraintsList != null) {
        //    foreach (LinearConstraint linearConstraint in linearConstraintsList) {
        //        Gizmos.color = Color.green;
        //        Gizmos.DrawLine(linearConstraint.pointL.position, linearConstraint.pointR.position);
        //    }
        //}


    }

#if UNITY_EDITOR
    [CustomEditor(typeof(Water2D))]
    public class Water2DEditor : Editor {
        public override void OnInspectorGUI() {
            base.OnInspectorGUI();
            Water2D water = (Water2D)target;

            if (GUILayout.Button("Reset")) {
                //water.particlesList = new List<Particle>();
                //water.particlesList.Add(new Particle());

                water.infoHolder = new InfoHolder();
                water.infoHolder.particlesV3 = new List<Vector3>();
                water.infoHolder.particlesV3.Add(Vector3.zero);
            }
        }
    }
#endif
    */
}
