#if UNITY_EDITOR
using UnityEditor;
#endif
using Newtonsoft.Json;
using System.Collections.Generic;
using UnityEngine;
using System;

public class Softbody2D : MonoBehaviour {
    const string SAVES_PATH = "\\ObjectsJson\\2D\\Softbody\\";
    [SerializeField] private string fileName;


    private List<Point> pointsList = new List<Point>();
    private List<Line> linesSurfaceList = new List<Line>();
    private List<Line> linesInnerList = new List<Line>();
    private List<Triangle> trianglesList = new List<Triangle>();
    private List<Line> mouseLinesList = new List<Line>();
    private List<Collision> collisionsList = new List<Collision>();

    [Header("Properties")]
    [SerializeField, Min(0)] private float mass = 1f;
    [SerializeField, Range(0f, 1f)] private float inverseStiffness = 0.5f;
    [SerializeField, Range(0.05f, 10f)] private float forceToTear = 0.2f;
    private bool torn = false;
    [SerializeField, Range(0f, 10f)] private float pressure = 1;
    private float defasultVolume;
    [SerializeField] private Vector2 gravity = Vector2.down * 9.8f;

    [Header("Computation")]
    [SerializeField, Range(0, 2)] private int volumeConstraintToUse = 1;
    [SerializeField, Min(1)] private uint substeps = 10; // splitting FixedUpdate
    [SerializeField, Min(1)] private uint iterations = 1; // of solving the constraints per substep


    [Header("To collide with")]
    [SerializeField] private List<StaticObject2D> staticObjectsList = new List<StaticObject2D>();


    private MeshFilter meshFilter;


    public float weirdA => inverseStiffness / Time.fixedDeltaTime / Time.fixedDeltaTime;
    float lambdaVolume;




    private void Awake() {
        LoadSoftbodyFromJson();
    }
    private void Update() {
        UpdateVisualMesh();

        if (Input.GetMouseButton(0) && mouseLinesList.Count == 0) {
            Vector2 mouseWordPosition = Camera.main.ScreenToWorldPoint(Input.mousePosition);
            // Trying to find a triangle witch points are around the mouseWorldPosition
            for (int triangle = 0; triangle < trianglesList.Count; triangle++) {
                float angle0 = Vector2.Dot(Vector2.Perpendicular(pointsList[trianglesList[triangle].firstPointId].position - pointsList[trianglesList[triangle].thirdPointId].position), mouseWordPosition - pointsList[trianglesList[triangle].thirdPointId].position);
                float angle1 = Vector2.Dot(Vector2.Perpendicular(pointsList[trianglesList[triangle].secondPointId].position - pointsList[trianglesList[triangle].firstPointId].position), mouseWordPosition - pointsList[trianglesList[triangle].firstPointId].position);
                float angle2 = Vector2.Dot(Vector2.Perpendicular(pointsList[trianglesList[triangle].thirdPointId].position - pointsList[trianglesList[triangle].secondPointId].position), mouseWordPosition - pointsList[trianglesList[triangle].secondPointId].position);
                if (angle0 < 0 && angle1 < 0 && angle2 < 0 ||
                    angle0 > 0 && angle1 > 0 && angle2 > 0) {
                    Point mousePointMass = new Point(this, mouseWordPosition, -1);
                    mousePointMass.softbody = this;
                    mouseLinesList.Add(new Line(this, mousePointMass, pointsList[trianglesList[triangle].firstPointId]));
                    mouseLinesList.Add(new Line(this, mousePointMass, pointsList[trianglesList[triangle].secondPointId]));
                    mouseLinesList.Add(new Line(this, mousePointMass, pointsList[trianglesList[triangle].thirdPointId]));
                    break;
                }
            }
            foreach (Line constraint in mouseLinesList) constraint.tearable = false;
        }
        if (Input.GetMouseButtonUp(0)) mouseLinesList.Clear();
    }

    private void FixedUpdate() {
        float deltaTime = Time.fixedDeltaTime / substeps;
        for (int substep = 0; substep < substeps; substep++) {
            foreach (Point pointMass in pointsList) {
                pointMass.previousPosition = pointMass.position;

                // Apply gravity
                pointMass.velocity += gravity * deltaTime;
                pointMass.position += pointMass.velocity * deltaTime;

                collisionsList.Clear();
                foreach (StaticObject2D staticObject in staticObjectsList) {
                    bool isInside = true;
                    float smallestDistance = float.NegativeInfinity;
                    Collision collision = new Collision();

                    Vector2 normal = Vector2.zero;
                    for (int i = 0; i < staticObject.lineList.Count; i++) {
                        List<Vector2> p = new List<Vector2> {
                        staticObject.lineList[i].pointL.position,
                        staticObject.lineList[i].pointR.position};

                        Vector2 lineNormal = Vector2.Perpendicular(p[1] - p[0]);
                        float cj = Vector2.Dot(pointMass.position - p[0], lineNormal);
                        if (cj >= 0) {
                            isInside = false;
                            break;
                        }

                        if (smallestDistance < cj) {
                            collision.point = pointMass.position - cj * lineNormal;
                            collision.normal = lineNormal;
                            smallestDistance = cj;

                            normal = lineNormal;
                        }
                    }
                    if (isInside) {
                        Debug.DrawRay(collision.point, normal);
                        collision.id = pointMass.id;
                        collisionsList.Add(collision);
                    }
                }
            }

            // Reset lambdas
            foreach (Line constraint in linesSurfaceList) constraint.lambda = 0;
            foreach (Line constraint in linesInnerList) constraint.lambda = 0;
            foreach (Line constraint in mouseLinesList) constraint.lambda = 0;
            lambdaVolume = 0;

            // Soling
            for (int iteration = 0; iteration < iterations; iteration++) {
                // bounds
                foreach (Point pointMass in pointsList) {
                    // floor
                    if (pointMass.position.y < GetFloorY_AtX(pointMass.position.x)) {
                        pointMass.position.y = GetFloorY_AtX(pointMass.position.x);
                        pointMass.velocity.y = 0;
                    }
                    // left wall
                    if (pointMass.position.x < -5) {
                        pointMass.position.x = -5;
                        pointMass.velocity.x = 0;
                    }
                    // right wall
                    if (pointMass.position.x > 5) {
                        pointMass.position.x = 5;
                        pointMass.velocity.x = 0;
                    }
                    // roof
                    if (pointMass.position.y > 5) {
                        pointMass.position.y = 5;
                        pointMass.velocity.y = 0;
                    }
                }

                /// ///////////////foreach (Collision collision in collisionsList) collision.

                // constraints
                foreach (Line constraint in linesSurfaceList) constraint.Solve();
                // mouse dragging
                foreach (Line constraint in mouseLinesList) {
                    constraint.pointL.position = Camera.main.ScreenToWorldPoint(Input.mousePosition);
                    constraint.pointL.velocity = Vector2.zero;
                    constraint.Solve();
                }
                foreach (Line constraint in linesSurfaceList) Line.VolumeConstraint(this, ref lambdaVolume);
            }

            foreach (Point pointMass in pointsList) {
                // Moving points
                pointMass.velocity = (pointMass.position - pointMass.previousPosition) / deltaTime;


                // Friction - I don't get it, don't blame me
                if (pointMass.position.y < GetFloorY_AtX(pointMass.position.x)) {
                    Vector2 floorNormal = Quaternion.Euler(Vector3.forward * 90) * new Vector2(1, GetFloorY_AtX(1)).normalized;
                    //Debug.DrawRay(Vector2.up * GetFloorY_AtX(0), floorNormal, Color.red, 0.1f);

                    Vector2 velocityNormal = Vector2.Dot(floorNormal, pointMass.velocity) * floorNormal;
                    Vector2 velicityTangential = pointMass.velocity - velocityNormal;
                    Vector2 deltaVelocity = -Vector2.Min(Vector2.one,
                        velicityTangential.magnitude * velicityTangential.normalized);

                    // no friction for now
                    //pointMass.velocity += deltaVelocity;
                }
            }
        }
    }


    private float GetFloorY_AtX(float xPos) {
        float ratio = -0.2f;
        return xPos * ratio - 3;
    }
    public float GetVolume() {
        float volume = 0.0f;
        for (int i = 0; i < linesSurfaceList.Count; i++) {
            float p0x = linesSurfaceList[i].pointL.position.x;
            float p0y = linesSurfaceList[i].pointL.position.y;
            float p1x = linesSurfaceList[i].pointR.position.x;
            float p1y = linesSurfaceList[i].pointR.position.y;

            volume += (p1x * p0y - p0x * p1y) / 2f;
        }
        return volume;
    }

    private void UpdateVisualMesh() {
        if (meshFilter == null) meshFilter = GetComponent<MeshFilter>();

        List<Vector3> vertices = new List<Vector3>();
        foreach (Point pointMass in pointsList) {
            vertices.Add(pointMass.position - (Vector2)transform.position);
        }
        List<int> triangles = new List<int>();
        foreach (Triangle triangle in trianglesList) {
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

        pointsList.Clear();
        linesSurfaceList.Clear();
        trianglesList.Clear();

        foreach (MeshCreator2D.Point point in data.pointList) {
            pointsList.Add(new Point(this, point.position + (Vector2)transform.position, pointsList.Count));
        }
        foreach (MeshCreator2D.Line line in data.linesSurfaceList) {
            linesSurfaceList.Add(new Line(this, pointsList[line.leftPointId], pointsList[line.rightPointId]));
        }
        foreach (MeshCreator2D.Triangle triangle in data.triangleList) {
            trianglesList.Add(new Triangle {
                firstPointId = triangle.firstPointId,
                secondPointId = triangle.secondPointId,
                thirdPointId = triangle.thirdPointId,
            });
        }

        defasultVolume = GetVolume();

        UpdateVisualMesh();
    }


    private class Point {
        public Point(Softbody2D softbody, Vector2 position, int id) {
            this.softbody = softbody;
            this.position = position;
            this.id = id;
        }

        public Softbody2D softbody;

        public int id;
        public Vector2 position;
        public Vector2 previousPosition;
        public Vector2 velocity;
        public float mass => softbody.mass / softbody.pointsList.Count;
    }
    private class Line {
        public Line(Softbody2D softbody, Point pointL, Point pointR) {
            this.softbody = softbody;
            this.pointL = pointL;
            this.pointR = pointR;
            defaultDistance = (pointL.position - pointR.position).magnitude;
        }

        public Softbody2D softbody;

        public Point pointL;
        public Point pointR;
        public float defaultDistance;
        public float inverseStiffness => softbody.inverseStiffness;
        public float lambda;
        public float deltaLambda;
        public bool tearable = true;
        public bool torn = false;

        //https://www.youtube.com/watch?v=MgmXJnR62uA&ab_channel=blackedout01 11:45
        public void Solve() {
            LinearConstraint(); // (18)
            //VolumeConstraint(); // Volume constraint
            ApplyResultOfCalculations(); // (17)
            TearCheck();
        }
        public void TearCheck() {
            if (tearable && Mathf.Abs(deltaLambda) > softbody.forceToTear * Time.deltaTime) {
                softbody.torn = torn = true;
                foreach (Triangle triangle in softbody.trianglesList) {
                    if (pointL.id == triangle.firstPointId || pointL.id == triangle.secondPointId || pointL.id == triangle.thirdPointId
                        && pointR.id == triangle.firstPointId || pointR.id == triangle.secondPointId || pointR.id == triangle.thirdPointId) {
                        softbody.trianglesList.Remove(triangle);
                        break;
                    }
                }
            }
        }
        public void LinearConstraint() { // (18)
            deltaLambda = (-((pointL.position - pointR.position).magnitude - defaultDistance) - softbody.weirdA * lambda) /
                (Mathf.Pow(pointL.mass, -1) + Mathf.Pow(pointR.mass, -1) + softbody.weirdA);
            lambda += deltaLambda;
        }
        public static void VolumeConstraint(Softbody2D softbody, ref float lambdaVolume) {
            if (softbody.torn) return;

            if (softbody.volumeConstraintToUse == 1) {
                float desiredPressure = softbody.GetVolume() - softbody.defasultVolume * softbody.pressure;

                List<Vector2> Jj = new List<Vector2>(new Vector2[softbody.pointsList.Count]);

                for (int j = 0; j < softbody.linesSurfaceList.Count; j++) {
                    Jj[softbody.linesSurfaceList[j].pointL.id] += Vector2.Perpendicular(softbody.linesSurfaceList[j].pointR.position - softbody.linesSurfaceList[j].pointL.position);
                    Jj[softbody.linesSurfaceList[j].pointR.id] += Vector2.Perpendicular(softbody.linesSurfaceList[j].pointR.position - softbody.linesSurfaceList[j].pointL.position);
                }

                for (int j = 0; j < softbody.pointsList.Count; j++) {
                    Debug.DrawRay(softbody.pointsList[j].position, Jj[j] * desiredPressure, Color.red);
                }
                float Denom = softbody.weirdA;
                for (int i = 0; i < softbody.pointsList.Count; i++) {
                    Denom += Mathf.Pow(softbody.pointsList[i].mass, -1) * Vector2.Dot(Jj[i], Jj[i]);
                }

                float volumeDeviation = (-desiredPressure - softbody.weirdA * lambdaVolume) / Denom;

                for (int i = 0; i < softbody.pointsList.Count; i++) {
                    softbody.pointsList[i].position += volumeDeviation * Mathf.Pow(softbody.pointsList[i].mass, -1) * Jj[i];
                }

                lambdaVolume += volumeDeviation;

            }
            //// garbage version
            //if (softbody.volumeConstraintToUse == 2) {
            //    float desiredPressure = softbody.GetVolume() - softbody.defasultVolume * softbody.pressure;
            //
            //    float lenghOfAllConstraints = 0f;
            //    foreach (var constraint in softbody.linesList) {
            //        if (volumetric == false) continue;
            //        lenghOfAllConstraints += (constraint.pointL.position - constraint.pointR.position).magnitude;
            //    }
            //
            //    Vector2 forceToConstraint = desiredPressure * Vector2.Perpendicular(pointL.position - pointR.position).normalized * (pointL.position - pointR.position).magnitude / lenghOfAllConstraints;
            //
            //    Debug.DrawRay(pointL.position, forceToConstraint);
            //    Debug.DrawRay(pointR.position, forceToConstraint);
            //
            //    pointL.position += forceToConstraint * Time.fixedDeltaTime;
            //    pointR.position += forceToConstraint * Time.fixedDeltaTime;
            //}
        }
        public void ApplyResultOfCalculations() { // (17)
            if (torn) return;
            pointR.position -= Mathf.Pow(pointR.mass, -1) * (pointL.position - pointR.position).normalized * deltaLambda;
            pointL.position += Mathf.Pow(pointL.mass, -1) * (pointL.position - pointR.position).normalized * deltaLambda;
        }
    }
    private class Triangle {
        public int firstPointId;
        public int secondPointId;
        public int thirdPointId;
    }
    private class Collision {
        public int id;
        public Vector2 point;
        public Vector2 normal;
        public float lambdaN;
        public float lambdaT;

        /*
        public void CollisionConstraint() {
            Vector2 p1;// contact point
            Vector2 normal;// = normal of collision plane
            float w = Mathf.Pow(mass, -1);// p0 inverse mass


            Vector2 p0 = position;
            float cj = Vector2.Dot(position - p1, normal);
            if (cj < 0) {
                float deltaLambdaN = -cj / w;
                Vector2 deltaX = -cj / w;
                LambdaN += deltaLambdaN;

                position += deltaX;
            }
        }
        */
    }






    //
    // INSPECTOR
    //
    [ExecuteInEditMode]
    private void OnDrawGizmos() {
        Gizmos.color = Color.green;

        foreach (Point point in pointsList) {
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(point.position, 0.05f);
        }

        foreach (Line constraint in linesSurfaceList) {
            if (constraint.torn) continue;
            // LinearConstraints
            Gizmos.color = Color.green;
            Gizmos.DrawLine(constraint.pointL.position, constraint.pointR.position);
            // constraint normal
            Gizmos.color = Color.white;
            Gizmos.DrawRay((constraint.pointL.position + constraint.pointR.position) / 2f, Vector2.Perpendicular(constraint.pointR.position - constraint.pointL.position).normalized / 5);
        }
        //floor formula y = x * angle
        Debug.DrawLine(new Vector2(-5, GetFloorY_AtX(-5)), new Vector2(5, GetFloorY_AtX(5)), Color.black);
    }


#if UNITY_EDITOR
    [CustomEditor(typeof(Softbody2D))]
    public class Softbody2DEditor : Editor {
        public override void OnInspectorGUI() {
            base.OnInspectorGUI();
            Softbody2D softbody = (Softbody2D)target;

            if (GUILayout.Button("Reset")) {
                softbody.LoadSoftbodyFromJson();
            }


            float angle = softbody.transform.eulerAngles.z * 3.14f / 180f;

            if (angle != 0) {
                Vector2 pivotPoint = Vector2.zero;
                foreach (Point pointMass in softbody.pointsList) pivotPoint += pointMass.position;
                pivotPoint /= softbody.pointsList.Count;

                float s = Mathf.Sin(angle);
                float c = Mathf.Cos(angle);
                foreach (Point pointMass in softbody.pointsList) {
                    pointMass.position -= pivotPoint;
                    pointMass.position = new Vector2(
                        c * pointMass.position.x + s * pointMass.position.y,
                        -s * pointMass.position.x + c * pointMass.position.y);
                    pointMass.position += pivotPoint;


                    pointMass.previousPosition -= pivotPoint;
                    pointMass.previousPosition = new Vector2(
                        c * pointMass.previousPosition.x + s * pointMass.previousPosition.y,
                        -s * pointMass.previousPosition.x + c * pointMass.previousPosition.y);
                    pointMass.previousPosition += pivotPoint;

                    softbody.UpdateVisualMesh();
                }
                softbody.transform.eulerAngles = Vector3.zero;
            }
        }
    }
#endif

}
