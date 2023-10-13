using System;
using System.Collections.Generic;
using System.Data;
using UnityEngine;
using UnityEngine.Experimental.GlobalIllumination;

public class MeshCreator2D : MonoBehaviour {
    const string SAVES_PATH = "\\ObjectsJson\\2D\\Softbody\\";
    [SerializeField] private string fileName;

    [Header("settings")]
    [SerializeField] private float gridSize = 0.5f;
    [SerializeField] private float thickness = 0.2f;

    [Header("visual")]
    [SerializeField] private bool showTriangles = true;
    [SerializeField] private bool showNonVolumetric = true;
    [SerializeField] private bool showGrid = true;

    // object properties
    private List<Point> points = new List<Point>();
    private List<Line> lines = new List<Line>();
    private List<Triangle> triangles = new List<Triangle>();




    // utilily
    private int selectedPoint = -1;
    private int movingPoint = -1;
    private List<int> newTrianglePoints = new List<int>();



    private void Update() {
        /// MOUSE
        Vector2 mouseWordPosition = Camera.main.ScreenToWorldPoint(Input.mousePosition);
        Vector2 mouseWordPositionToGrid = mouseWordPosition + new Vector2(
            mouseWordPosition.x % gridSize < gridSize / 2 ? -mouseWordPosition.x % gridSize : gridSize - mouseWordPosition.x % gridSize,
            mouseWordPosition.y % gridSize < gridSize / 2 ? -mouseWordPosition.y % gridSize : gridSize - mouseWordPosition.y % gridSize);


        //right
        if (Input.GetMouseButtonDown(1)) {
            // Delete point
            for (int i = 0; i < points.Count; i++) if ((points[i].position - mouseWordPosition).magnitude < thickness * gridSize) {
                    points.RemoveAt(i);
                    return;
                }

            // Add point
            bool canPlacePoint = true;
            for (int i = 0; i < points.Count; i++) if ((points[i].position - mouseWordPositionToGrid).magnitude < thickness * gridSize) canPlacePoint = false;
            if (canPlacePoint) points.Add(new Point {
                position = mouseWordPositionToGrid,
            });
        }
        //right END

        //modle
        if (Input.GetMouseButtonDown(2)) {
            for (int i = 0; i < points.Count; i++) if ((points[i].position - mouseWordPosition).magnitude < thickness * gridSize) {
                    movingPoint = i;
                    return;
                }
        }
        if (Input.GetMouseButtonDown(2)) {
            // set a line volumetric
            foreach (Line line in lines) {
                Vector2 leftPointPosition = points[line.leftPointId].position;
                Vector2 rightPointPosition = points[line.rightPointId].position;
                // thickness check
                if (Math.Abs(Vector2.Dot(Vector2.Perpendicular(leftPointPosition - rightPointPosition).normalized, mouseWordPosition - (leftPointPosition + rightPointPosition) / 2)) < thickness * gridSize) {
                    // lenght check
                    if (Vector2.Dot(mouseWordPosition - leftPointPosition, rightPointPosition - leftPointPosition) > 0 &&
                        Vector2.Dot(mouseWordPosition - rightPointPosition, leftPointPosition - rightPointPosition) > 0) {

                        line.volumetric = !line.volumetric;
                        return;
                    }
                }
            }
        }
        if (movingPoint != -1) {
            points[movingPoint].position = mouseWordPositionToGrid;
        }
        if (Input.GetMouseButtonUp(2)) movingPoint = -1;
        //modle END

        //left
        if (Input.GetMouseButtonDown(0)) {
            // grabbing point that is near enough
            for (int i = 0; i < points.Count; i++) if ((points[i].position - mouseWordPosition).magnitude < thickness * gridSize) selectedPoint = i;
            // if clicked on a point
            if (selectedPoint > -1) return;


            // flip a line
            foreach (Line line in lines) {
                Vector2 leftPointPosition = points[line.leftPointId].position;
                Vector2 rightPointPosition = points[line.rightPointId].position;
                // thickness check
                if (Math.Abs(Vector2.Dot(Vector2.Perpendicular(leftPointPosition - rightPointPosition).normalized, mouseWordPosition - (leftPointPosition + rightPointPosition) / 2)) < thickness * gridSize) {
                    // lenght check
                    if (Vector2.Dot(mouseWordPosition - leftPointPosition, rightPointPosition - leftPointPosition) > 0 &&
                        Vector2.Dot(mouseWordPosition - rightPointPosition, leftPointPosition - rightPointPosition) > 0) {

                        int idHolder = line.leftPointId;
                        line.leftPointId = line.rightPointId;
                        line.rightPointId = idHolder;
                        return;
                    }
                }
            }

        }
        if (Input.GetMouseButton(0)) {
            for (int i = 0; i < points.Count; i++) if (i == selectedPoint) Debug.DrawLine(points[i].position, mouseWordPosition);
        }
        if (Input.GetMouseButtonUp(0)) {
            // connecting two points with a line
            int pointToConnectTo = -1;
            for (int i = 0; i < points.Count; i++) if ((points[i].position - mouseWordPosition).magnitude < thickness * gridSize) pointToConnectTo = i;

            // cancel new triangle. cause - do and point
            if (selectedPoint == -1) newTrianglePoints.Clear();
            // add point to triangle
            else if (pointToConnectTo == selectedPoint) {
                newTrianglePoints.Add(selectedPoint);

                if (newTrianglePoints.Count == 3) {
                    triangles.Add(new Triangle {
                        firstPointId = newTrianglePoints[0],
                        secondPointId = newTrianglePoints[1],
                        thirdPointId = newTrianglePoints[2],
                    });
                    newTrianglePoints.Clear();
                }
            }
            // connect points with line
            else if (pointToConnectTo > -1 && selectedPoint > -1) {
                lines.Add(new Line {
                    leftPointId = selectedPoint,
                    rightPointId = pointToConnectTo,
                });

                // cancel triangle creation
                newTrianglePoints.Clear();
            }

            // reset point selection
            selectedPoint = -1;
        }
        //left END


        // Reset lines and triangles
        if (Input.GetKeyDown(KeyCode.R)) {
            lines.Clear();
            triangles.Clear();
        }
        // Save to json
        if (Input.GetKeyDown(KeyCode.S)) {
            SaveMeshToJson();
        }
        // Load from json
        if (Input.GetKeyDown(KeyCode.L)) {
            LoadMeshFromJson();
        }
        // Info
        if (Input.GetKeyDown(KeyCode.I)) {
            Debug.Log("points.Count: " + points.Count);
            Debug.Log("lines.Count: " + lines.Count);
            Debug.Log("triangles.Count:" + triangles.Count);
        }


    }



    private void SaveMeshToJson() {
        SaveSystem.WriteJson(SAVES_PATH + fileName + ".json", SaveSystem.SerializeJson(new Softbody2DJsonObject {
            pointList = points,
            lineList = lines,
            triangleList = triangles,
        }));
    }
    private void LoadMeshFromJson() {
        Softbody2DJsonObject data = SaveSystem.DeserializeJson<Softbody2DJsonObject>(SaveSystem.ReadJson(SAVES_PATH + fileName + ".json"));

        points = data.pointList;
        lines = data.lineList;
        triangles = data.triangleList;
    }




    [Serializable]
    public class Point {
        public Vector2 position;
    }
    [Serializable]
    public class Line {
        public int leftPointId;
        public int rightPointId;
        public bool volumetric = true;
    }
    [Serializable]
    public class Triangle {
        public int firstPointId;
        public int secondPointId;
        public int thirdPointId;
    }
    [Serializable]
    public class Softbody2DJsonObject {
        public List<Point> pointList;
        public List<Line> lineList;
        public List<Triangle> triangleList;
    }




    //
    // Visual
    //
    [ExecuteInEditMode]
    private void OnDrawGizmos() {

        // points
        foreach (Point point in points) {
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(point.position, thickness * gridSize);
        }

        // lines
        foreach (Line line in lines) {
            Gizmos.color = Color.green;
            Vector2 leftPointPosition = new Vector2();
            Vector2 rightPointPosition = new Vector2();
            for (int i = 0; i < points.Count; i++) {
                if (line.leftPointId == i) leftPointPosition = points[i].position;
                if (line.rightPointId == i) rightPointPosition = points[i].position;
            }
            if (line.volumetric) {
                Vector2 normalPosition = Vector2.Perpendicular(rightPointPosition - leftPointPosition).normalized * thickness;
                // lines
                Gizmos.DrawLine(leftPointPosition + normalPosition, rightPointPosition + normalPosition);
            }
            else if (showNonVolumetric) {
                Gizmos.color = Color.red;
                Gizmos.DrawLine(leftPointPosition, rightPointPosition);
            }
        }

        // triangles
        if (showTriangles)
            foreach (Triangle tri in triangles) {
                Gizmos.color = Color.blue;

                Gizmos.DrawLine((points[tri.firstPointId].position + points[tri.secondPointId].position + points[tri.thirdPointId].position) / 3, points[tri.firstPointId].position);
                Gizmos.DrawLine((points[tri.firstPointId].position + points[tri.secondPointId].position + points[tri.thirdPointId].position) / 3, points[tri.secondPointId].position);
                Gizmos.DrawLine((points[tri.firstPointId].position + points[tri.secondPointId].position + points[tri.thirdPointId].position) / 3, points[tri.thirdPointId].position);
            }

        // making triangle
        foreach (int point in newTrianglePoints) {
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(points[point].position, thickness * gridSize * 1.1f);
        }

        // grid
        if (showGrid) {
            Gizmos.color = Color.white * 0.5f;
            int size = 100;
            for (int r = -size / 2; r < size / 2; r++) Gizmos.DrawLine(Vector2.right * -size / 2 + Vector2.up * r * gridSize, Vector2.right * size / 2 + Vector2.up * r * gridSize);
            for (int c = -size / 2; c < size / 2; c++) Gizmos.DrawLine(Vector2.up * -size / 2 + Vector2.right * c * gridSize, Vector2.up * size / 2 + Vector2.right * c * gridSize);
        }

    }

}
