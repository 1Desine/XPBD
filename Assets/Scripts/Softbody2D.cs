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


    private List<PointMass> pointMassList = new List<PointMass>();
    private List<LinearConstraint> linearConstraintList = new List<LinearConstraint>();
    private List<Triangle> triangleList = new List<Triangle>();
    private List<int> tornLinearConstraintsIndexesList = new List<int>();
    private List<LinearConstraint> linearMouseDragConstraintsList = new List<LinearConstraint>();

    [Header("Properties")]
    [SerializeField, Min(0)] private float mass = 1f;
    [SerializeField, Range(0f, 1f)] private float inverseStiffness = 0.5f;
    [SerializeField, Range(0.01f, 10f)] private float forceToTear = 0.2f;
    private bool torn = false;
    [SerializeField, Range(0f, 10f)] private float pressure = 1;
    private float defasultVolume;
    [SerializeField] private Vector2 gravity = Vector2.down * 9.8f;

    [Header("Computation")]
    [SerializeField, Range(0, 2)] private int volumeConstraintToUse = 1;
    [SerializeField, Min(1)] private uint substeps = 10; // splitting FixedUpdate
    [SerializeField, Min(1)] private uint iterations = 1; // of solving the constraints per substep


    private MeshFilter meshFilter;



    private void Awake() {
        LoadSoftbodyFromJson();
        SetupValuesForConstraintsAndPoints();
        defasultVolume = GetVolume();
    }
    private void Update() {
        //UpdateVisualMesh();

        if (Input.GetMouseButton(0) && linearMouseDragConstraintsList.Count == 0) {
            Vector2 mouseWordPosition = Camera.main.ScreenToWorldPoint(Input.mousePosition);
            // Trying to find a triangle witch points are around the mouseWorldPosition
            for (int triangle = 0; triangle < triangleList.Count; triangle++) {
                float angle0 = Vector2.Dot(Vector2.Perpendicular(pointMassList[triangleList[triangle].firstPointId].position - pointMassList[triangleList[triangle].thirdPointId].position), mouseWordPosition - pointMassList[triangleList[triangle].thirdPointId].position);
                float angle1 = Vector2.Dot(Vector2.Perpendicular(pointMassList[triangleList[triangle].secondPointId].position - pointMassList[triangleList[triangle].firstPointId].position), mouseWordPosition - pointMassList[triangleList[triangle].firstPointId].position);
                float angle2 = Vector2.Dot(Vector2.Perpendicular(pointMassList[triangleList[triangle].thirdPointId].position - pointMassList[triangleList[triangle].secondPointId].position), mouseWordPosition - pointMassList[triangleList[triangle].secondPointId].position);
                if (angle0 < 0 && angle1 < 0 && angle2 < 0 ||
                    angle0 > 0 && angle1 > 0 && angle2 > 0) {
                    PointMass mousePointMass = new PointMass(mouseWordPosition, -1);
                    mousePointMass.softbody = this;
                    linearMouseDragConstraintsList.Add(new LinearConstraint(mousePointMass, pointMassList[triangleList[triangle].firstPointId], false));
                    linearMouseDragConstraintsList.Add(new LinearConstraint(mousePointMass, pointMassList[triangleList[triangle].secondPointId], false));
                    linearMouseDragConstraintsList.Add(new LinearConstraint(mousePointMass, pointMassList[triangleList[triangle].thirdPointId], false));
                    break;
                }
            }
            foreach (LinearConstraint constraint in linearMouseDragConstraintsList) {
                constraint.tearable = false;
                constraint.Setup();
                constraint.softbody = this;
            }
        }
        if (Input.GetMouseButtonUp(0)) linearMouseDragConstraintsList.Clear();


        // VISUAL

        //points are connected clockwise
        foreach (LinearConstraint constraint in linearConstraintList) {
            // LinearConstraints
            Debug.DrawLine(constraint.pointL.position, constraint.pointR.position, Color.black);
            // constraint nornal
            //Debug.DrawRay((constraint.point0.position + constraint.point1.position) / 2f, Vector2.Perpendicular(constraint.point1.position - constraint.point0.position), Color.black);

            // Constraints perpendicular vectors
            //Debug.DrawRay(linearConstraintsList[i].point0.position, Vector2.Perpendicular(linearConstraintsList[i].point1.position - linearConstraintsList[i].point0.position).normalized);
            //Debug.DrawRay(linearConstraintsList[i].point1.position, Vector2.Perpendicular(linearConstraintsList[i].point1.position - linearConstraintsList[i].point0.position).normalized);
        }
        //floor formula y = x * angle
        Debug.DrawLine(new Vector2(-5, GetFloorY_AtX(-5)), new Vector2(5, GetFloorY_AtX(5)), Color.black);
        // Update visual points positions
        //for (int p = 0; p < pointMassList.Count; p++) pointMassVisualList[p].position = pointMassList[p].position;
    }
    private void FixedUpdate() {
        float deltaTime = Time.fixedDeltaTime / substeps;
        for (int substep = 0; substep < substeps; substep++) {
            foreach (PointMass pointMass in pointMassList) {
                pointMass.previousPosition = pointMass.position;

                // Apply gravity
                pointMass.velocity += gravity * deltaTime;
                pointMass.position += pointMass.velocity * deltaTime;
            }

            // Reset lambdas
            foreach (LinearConstraint constraint in linearConstraintList) constraint.lambda = 0;
            foreach (LinearConstraint constraint in linearMouseDragConstraintsList) constraint.lambda = 0;

            // Soling
            //https://www.youtube.com/watch?v=MgmXJnR62uA&ab_channel=blackedout01 11:45
            for (int iteration = 0; iteration < iterations; iteration++) {
                // bounds
                foreach (PointMass pointMass in pointMassList) {
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

                // constraints
                foreach (LinearConstraint constraint in linearConstraintList) constraint.Solve();
                // mouse dragging
                foreach (LinearConstraint constraint in linearMouseDragConstraintsList) {
                    constraint.pointL.position = Camera.main.ScreenToWorldPoint(Input.mousePosition);
                    constraint.pointL.velocity = Vector2.zero;
                    constraint.Solve();
                }
                if (tornLinearConstraintsIndexesList.Count != 0) {
                    foreach (int tornConstraint in tornLinearConstraintsIndexesList) {
                        linearConstraintList.RemoveAt(tornConstraint);
                    }
                    tornLinearConstraintsIndexesList.Clear();
                }
            }

            foreach (PointMass pointMass in pointMassList) {
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
        for (int i = 0; i < linearConstraintList.Count; i++) {
            if (linearConstraintList[i].volumetric == false) continue;
            float p0x = linearConstraintList[i].pointL.position.x;
            float p0y = linearConstraintList[i].pointL.position.y;
            float p1x = linearConstraintList[i].pointR.position.x;
            float p1y = linearConstraintList[i].pointR.position.y;

            volume += (p1x * p0y - p0x * p1y) / 2f;
        }
        return volume;
    }

    private void SetupValuesForConstraintsAndPoints() {
        foreach (LinearConstraint constraint in linearConstraintList) {
            constraint.softbody = this;

            constraint.Setup();
        }
        foreach (PointMass pointMass in pointMassList) pointMass.softbody = this;
    }
    private void UpdateVisualMesh() {
        if (meshFilter == null) meshFilter = GetComponent<MeshFilter>();

        List<Vector3> vertices = new List<Vector3>();
        foreach (PointMass pointMass in pointMassList) {
            vertices.Add(pointMass.position);
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

        pointMassList.Clear();
        linearConstraintList.Clear();
        triangleList.Clear();

        foreach (MeshCreator2D.Point point in data.pointList) {
            pointMassList.Add(new PointMass(point.position, pointMassList.Count));
        }
        foreach (MeshCreator2D.Line line in data.lineList) {
            linearConstraintList.Add(new LinearConstraint(pointMassList[line.leftPointId], pointMassList[line.rightPointId], line.volumetric));
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


    ///////////////////
    ///////////////////
    private class PointMass {
        public PointMass(Vector2 position, int id) {
            this.position = position;
            this.id = id;
        }

        public Softbody2D softbody;

        public int id;
        public Vector2 position;
        public Vector2 previousPosition;
        public Vector2 velocity;
        public float mass { get { return softbody.mass / softbody.pointMassList.Count; } }
    }
    ///////////////////
    ///////////////////
    private class LinearConstraint {
        public LinearConstraint(PointMass pointL, PointMass pointR, bool volumetric) {
            this.pointL = pointL;
            this.pointR = pointR;
            this.volumetric = volumetric;
            Setup();
        }
        public void Setup() {
            defaultDistance = (pointL.position - pointR.position).magnitude;
        }

        public Softbody2D softbody;

        public PointMass pointL;
        public PointMass pointR;
        public float defaultDistance;
        public float inverseStiffness { get { return softbody.inverseStiffness; } }
        public float weirdA { get { return inverseStiffness / Time.fixedDeltaTime / Time.fixedDeltaTime; } }
        public float lambda;
        public float deltaLambda;
        public bool volumetric;
        public bool tearable = true;

        public void Solve() {
            ComputeDeltaLambda(); // (18)
            if (volumetric && softbody.torn == false) ComputeVolumeConstraint(); // Volume constraint
            ComputeDeltaDistance(); // (17)
        }
        public void ComputeDeltaLambda() { // (18)
            deltaLambda = (-((pointL.position - pointR.position).magnitude - defaultDistance) - weirdA * lambda) /
                (Mathf.Pow(pointL.mass, -1) + Mathf.Pow(pointR.mass, -1) + weirdA);
            lambda += deltaLambda;

            if (tearable && Mathf.Abs(deltaLambda) > softbody.forceToTear * Time.deltaTime) {
                softbody.tornLinearConstraintsIndexesList.Add(softbody.linearConstraintList.IndexOf(this));
                softbody.tornLinearConstraintsIndexesList.Sort();
                softbody.tornLinearConstraintsIndexesList.Reverse();
                softbody.torn = true;
            }
        }
        public void ComputeVolumeConstraint() {
            if (softbody.volumeConstraintToUse == 1) {
                float desiredPressure = softbody.GetVolume() - softbody.defasultVolume * softbody.pressure;

                List<Vector2> Jj = new List<Vector2>(new Vector2[softbody.pointMassList.Count]);

                for (int j = 0; j < softbody.linearConstraintList.Count; j++) {
                    if (softbody.linearConstraintList[j].volumetric == false) continue;
                    Jj[softbody.linearConstraintList[j].pointL.id] += Vector2.Perpendicular(softbody.linearConstraintList[j].pointR.position - softbody.linearConstraintList[j].pointL.position);
                    Jj[softbody.linearConstraintList[j].pointR.id] += Vector2.Perpendicular(softbody.linearConstraintList[j].pointR.position - softbody.linearConstraintList[j].pointL.position);
                }

                for (int j = 0; j < softbody.pointMassList.Count; j++) {
                    Debug.DrawRay(softbody.pointMassList[j].position, Jj[j] * desiredPressure, Color.red);
                }
                float Denom = weirdA;
                for (int i = 0; i < softbody.pointMassList.Count; i++) {
                    Denom += Mathf.Pow(softbody.pointMassList[i].mass, -1) * Vector2.Dot(Jj[i], Jj[i]);
                }

                float volumeDeviation = (-desiredPressure - weirdA * lambda) / Denom;

                for (int i = 0; i < softbody.pointMassList.Count; i++) {
                    softbody.pointMassList[i].position += volumeDeviation * Mathf.Pow(softbody.pointMassList[i].mass, -1) * Jj[i];
                }

                lambda += volumeDeviation;

            }
            // garbage version
            if (softbody.volumeConstraintToUse == 2) {
                float desiredPressure = softbody.GetVolume() - softbody.defasultVolume * softbody.pressure;

                float lenghOfAllConstraints = 0f;
                foreach (var constraint in softbody.linearConstraintList) {
                    if (volumetric == false) continue;
                    lenghOfAllConstraints += (constraint.pointL.position - constraint.pointR.position).magnitude;
                }

                Vector2 forceToConstraint = desiredPressure * Vector2.Perpendicular(pointL.position - pointR.position).normalized * (pointL.position - pointR.position).magnitude / lenghOfAllConstraints;

                Debug.DrawRay(pointL.position, forceToConstraint);
                Debug.DrawRay(pointR.position, forceToConstraint);

                pointL.position += forceToConstraint * Time.fixedDeltaTime;
                pointR.position += forceToConstraint * Time.fixedDeltaTime;
            }
        }
        public void ComputeDeltaDistance() { // (17)
            pointL.position += Mathf.Pow(pointL.mass, -1) * (pointL.position - pointR.position).normalized * deltaLambda;
            pointR.position -= Mathf.Pow(pointR.mass, -1) * (pointL.position - pointR.position).normalized * deltaLambda;
        }
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

        if (pointMassList != null) {
            foreach (PointMass point in pointMassList) {
                Gizmos.color = Color.green;
                Gizmos.DrawSphere(point.position, 0.05f);
            }
        }

        if (linearConstraintList != null) {
            foreach (LinearConstraint linearConstraint in linearConstraintList) {
                Gizmos.color = Color.green;
                Gizmos.DrawLine(linearConstraint.pointL.position, linearConstraint.pointR.position);
            }
        }
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
        }
    }
#endif


}
