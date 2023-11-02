
    #if UNITY_EDITOR
using UnityEditor;
#endif
using Newtonsoft.Json;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;

public class StaticObject2D : MonoBehaviour {
    const string SAVES_PATH = "\\ObjectsJson\\2D\\Softbody\\";
    [SerializeField] private string fileName;

    private List<Point> pointList = new List<Point>();
    public List<Line> lineList = new List<Line>();
    private List<Triangle> triangleList = new List<Triangle>();

    private MeshFilter meshFilter;


    private void Awake() {
        LoadSoftbodyFromJson();
    }
    private void UpdateVisualMesh() {
        if (meshFilter == null) meshFilter = GetComponent<MeshFilter>();

        List<Vector3> vertices = new List<Vector3>();
        foreach (Point pointMass in pointList) {
            vertices.Add(pointMass.position - (Vector2)transform.position);
        }
        List<int> triangles = new List<int>();
        foreach (Triangle triangle in triangleList) {
            triangles.Add(triangle.firstPointId);
            triangles.Add(triangle.secondPointId);
            triangles.Add(triangle.thirdPointId);

            triangles.Add(triangle.secondPointId);
            triangles.Add(triangle.firstPointId);
            triangles.Add(triangle.thirdPointId);
        }

        Mesh meshToApply = new Mesh();
        meshToApply.vertices = vertices.ToArray();
        meshToApply.triangles = triangles.ToArray();

        meshFilter.mesh = meshToApply;
    }
    private void LoadSoftbodyFromJson() {
        MeshCreator2D.Softbody2DJsonObject data = SaveSystem.DeserializeJson<MeshCreator2D.Softbody2DJsonObject>(SaveSystem.ReadJson(SAVES_PATH + fileName + ".json"));

        pointList.Clear();
        lineList.Clear();
        triangleList.Clear();

        foreach (MeshCreator2D.Point point in data.pointList) {
            pointList.Add(new Point(this, point.position + (Vector2)transform.position, pointList.Count));
        }
        foreach (MeshCreator2D.Line line in data.linesSurfaceList) {
            lineList.Add(new Line(this, pointList[line.leftPointId], pointList[line.rightPointId]));
        }
        foreach (MeshCreator2D.Triangle triangle in data.triangleList) {
            triangleList.Add(new Triangle {
                firstPointId = triangle.firstPointId,
                secondPointId = triangle.secondPointId,
                thirdPointId = triangle.thirdPointId,
            });
        }

        UpdateVisualMesh();
    }


    public class Point {
        public Point(StaticObject2D softbody, Vector2 position, int id) {
            this.softbody = softbody;
            this.position = position;
            this.id = id;
        }

        public StaticObject2D softbody;

        public int id;
        public Vector2 position;
    }
    public class Line {
        public Line(StaticObject2D staticObjec, Point pointL, Point pointR) {
            this.staticObjec= staticObjec;
            this.pointL = pointL;
            this.pointR = pointR;
        }
        public StaticObject2D staticObjec;

        public Point pointL;
        public Point pointR;
    }
    private class Triangle {
        public int firstPointId;
        public int secondPointId;
        public int thirdPointId;
    }




    //
    // INSPECTOR
    //
    [ExecuteInEditMode]
    private void OnDrawGizmos() {
        Gizmos.color = Color.green;

        foreach (Line constraint in lineList) {
            // LinearConstraints
            Gizmos.color = Color.green;
            Gizmos.DrawLine(constraint.pointL.position, constraint.pointR.position);
        }
    }


#if UNITY_EDITOR
    [CustomEditor(typeof(StaticObject2D))]
    public class StaticObject2DEditor : Editor {
        public override void OnInspectorGUI() {
            base.OnInspectorGUI();
            StaticObject2D softbody = (StaticObject2D)target;

            if (GUILayout.Button("Reset")) {
                softbody.LoadSoftbodyFromJson();
            }

            float angle = softbody.transform.eulerAngles.z * 3.14f / 180f;

            if (angle != 0) {
                Vector2 pivotPoint = Vector2.zero;
                foreach (Point pointMass in softbody.pointList) pivotPoint += pointMass.position;
                pivotPoint /= softbody.pointList.Count;

                float s = Mathf.Sin(angle);
                float c = Mathf.Cos(angle);
                foreach (Point pointMass in softbody.pointList) {
                    pointMass.position -= pivotPoint;
                    pointMass.position = new Vector2(
                        c * pointMass.position.x + s * pointMass.position.y,
                        -s * pointMass.position.x + c * pointMass.position.y);
                    pointMass.position += pivotPoint;
                }
                softbody.transform.eulerAngles = Vector3.zero;
            }
        }
    }
#endif

}
