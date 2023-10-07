#if UNITY_EDITOR
using UnityEditor;
#endif
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.Rendering;

public class Softbody2 : MonoBehaviour {
    [SerializeField] private Mesh mesh;
    [SerializeField] private bool useMesh;

    private List<PointMass> pointMassList = new List<PointMass>();
    private List<LinearConstraint> linearConstraintsList = new List<LinearConstraint>();
    private List<LinearConstraint> linearMouseDragConstraintsList = new List<LinearConstraint>();
    private List<Peak> peaksList = new List<Peak>();

    [SerializeField] private float mass = 1f;
    [Range(0f, 1f)]
    [SerializeField] private float inverseStiffness = 0.5f;
    private float previousInverseStiffness;
    [SerializeField] private Vector2 gravity = Vector2.down * 9.81f;
    [Range(0f, 100f)]
    [SerializeField] private float pressure = 1;
    private float defasultVolume;

    [SerializeField] private uint substeps = 10; // splitting FixedUpdate
    [SerializeField] private uint iterations = 1; // of solving the constraints per substep

    private MeshFilter meshFilter;

    private void Awake() {
        meshFilter = GetComponent<MeshFilter>();

        if (useMesh) LoadObjectFromMesh();

        // inner constraints remove
        linearConstraintsList.RemoveAt(4);
        linearConstraintsList.RemoveAt(3);
        linearConstraintsList.RemoveAt(1);

        //linearConstraintsList[4].volumetric = false;
        //linearConstraintsList[3].volumetric = false;
        //linearConstraintsList[1].volumetric = false;

        defasultVolume = GetVolume();

        foreach (var pointMass in pointMassList) {
            PointMass leftPoint = null;
            PointMass rightPoint = null;
            foreach (var linearConstraint in linearConstraintsList) {
                if (pointMass == linearConstraint.point0) rightPoint = linearConstraint.point1;
                if (pointMass == linearConstraint.point1) leftPoint = linearConstraint.point0;
            }
            if (leftPoint != null && rightPoint != null) peaksList.Add(new Peak {
                pointL = leftPoint,
                pointR = rightPoint,
            });
        }
    }
    private void Update() {
        Mesh meshToApply = new Mesh();
        meshToApply.vertices = mesh.vertices;
        meshToApply.triangles = mesh.triangles;

        Vector2[] newUVs = new Vector2[mesh.vertices.Length];
        Vector3[] newVertices = new Vector3[mesh.vertices.Length];
        for (int v = 0; v < mesh.vertices.Length; v++) {
            newVertices[v] = pointMassList[v].position;
            newUVs[v] = pointMassList[v].position;
        }
        meshToApply.vertices = newVertices;
        meshToApply.uv = newUVs;
        meshFilter.mesh = meshToApply;

        if (Input.GetMouseButtonDown(0)) {
            Vector2 mouseWordPosition = Camera.main.ScreenToWorldPoint(Input.mousePosition);
            // Trying to find a triangle witch points are around the mouseWorldPosition
            for (int posInTriangles = 0; posInTriangles < mesh.triangles.Length; posInTriangles += 3) {
                float angle0 = Vector2.Dot(Vector2.Perpendicular(pointMassList[mesh.triangles[posInTriangles + 0]].position - pointMassList[mesh.triangles[posInTriangles + 2]].position), mouseWordPosition - pointMassList[mesh.triangles[posInTriangles + 2]].position);
                float angle1 = Vector2.Dot(Vector2.Perpendicular(pointMassList[mesh.triangles[posInTriangles + 1]].position - pointMassList[mesh.triangles[posInTriangles + 0]].position), mouseWordPosition - pointMassList[mesh.triangles[posInTriangles + 0]].position);
                float angle2 = Vector2.Dot(Vector2.Perpendicular(pointMassList[mesh.triangles[posInTriangles + 2]].position - pointMassList[mesh.triangles[posInTriangles + 1]].position), mouseWordPosition - pointMassList[mesh.triangles[posInTriangles + 1]].position);
                if (angle0 < 0 && angle1 < 0 && angle2 < 0 ||
                    angle0 > 0 && angle1 > 0 && angle2 > 0) {
                    linearMouseDragConstraintsList.Add(new LinearConstraint(new PointMass(mouseWordPosition, this), pointMassList[mesh.triangles[posInTriangles + 0]], 0.5f, false, this));
                    linearMouseDragConstraintsList.Add(new LinearConstraint(new PointMass(mouseWordPosition, this), pointMassList[mesh.triangles[posInTriangles + 1]], 0.5f, false, this));
                    linearMouseDragConstraintsList.Add(new LinearConstraint(new PointMass(mouseWordPosition, this), pointMassList[mesh.triangles[posInTriangles + 2]], 0.5f, false, this));
                    break;
                }
            }
        }
        if (Input.GetMouseButtonUp(0)) linearMouseDragConstraintsList.Clear();


        /////////////// VISUAL ///////////////

        //points are connected clockwise
        foreach (LinearConstraint constraint in linearConstraintsList) {
            // LinearConstraints
            Debug.DrawLine(constraint.point0.position, constraint.point1.position, Color.black);
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
        if (previousInverseStiffness != inverseStiffness) foreach (LinearConstraint constraint in linearConstraintsList) constraint.SetInverseStiffness(inverseStiffness);
        previousInverseStiffness = inverseStiffness;

        float deltaTime = Time.fixedDeltaTime / substeps;
        for (int substep = 0; substep < substeps; substep++) {
            // Apply gravity
            foreach (PointMass pointMass in pointMassList) {
                pointMass.previousPosition = pointMass.position;

                pointMass.velocity += gravity * deltaTime;
                pointMass.position += pointMass.velocity * deltaTime;
            }

            // Reset lambdas
            foreach (LinearConstraint constraint in linearConstraintsList) constraint.lambda = 0;
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
                foreach (LinearConstraint constraint in linearConstraintsList) constraint.Solve();
                // mouse dragging
                foreach (LinearConstraint constraint in linearMouseDragConstraintsList) {
                    constraint.point0.position = Camera.main.ScreenToWorldPoint(Input.mousePosition);
                    constraint.point0.velocity = Vector2.zero;
                    constraint.Solve();
                }
            }

            // Moving points
            foreach (PointMass pointMass in pointMassList) {
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
        float ratio = 0;// -0.2f;
        return xPos * ratio - 3;
    }

    private void LoadObjectFromMesh() {
        // Initiate pointMassList from mesh
        foreach (var vertex in mesh.vertices) {
            pointMassList.Add(new PointMass(vertex, this));
            //pointMassVisualList.Add(Instantiate(visualPrefab, transform));
        }
        // Sign in nearest points
        for (int posInTriangles = 0; posInTriangles < mesh.triangles.Length; posInTriangles += 3) {
            PointMass v0 = pointMassList[0]; // HOLDER
            PointMass v1 = pointMassList[0]; // HOLDER
            PointMass v2 = pointMassList[0]; // HOLDER
            foreach (var pointMass in pointMassList) {
                if (pointMass.position == (Vector2)mesh.vertices[mesh.triangles[posInTriangles + 0]]) v0 = pointMass;
                if (pointMass.position == (Vector2)mesh.vertices[mesh.triangles[posInTriangles + 1]]) v1 = pointMass;
                if (pointMass.position == (Vector2)mesh.vertices[mesh.triangles[posInTriangles + 2]]) v2 = pointMass;
            }

            // Check from pointMass, does it have connections with other pointMasses in this triangle
            if (PointsAreConnected(v0, v1) == false) linearConstraintsList.Add(new LinearConstraint(v0, v1, inverseStiffness, true, this));
            if (PointsAreConnected(v1, v2) == false) linearConstraintsList.Add(new LinearConstraint(v1, v2, inverseStiffness, true, this));
            if (PointsAreConnected(v2, v0) == false) linearConstraintsList.Add(new LinearConstraint(v2, v0, inverseStiffness, true, this));
        }
    }
    private bool PointsAreConnected(PointMass point0, PointMass point1) {
        foreach (var constraint in linearConstraintsList) {
            if (constraint.point0 == point0 && constraint.point1 == point1
             || constraint.point0 == point1 && constraint.point1 == point0) return true;
        }
        return false;
    }

    public float GetVolume() {
        float volume = 0.0f;
        for (int i = 0; i < linearConstraintsList.Count; i++) {
            if (linearConstraintsList[i].volumetric == false) continue;
            float p0x = linearConstraintsList[i].point0.position.x;
            float p0y = linearConstraintsList[i].point0.position.y;
            float p1x = linearConstraintsList[i].point1.position.x;
            float p1y = linearConstraintsList[i].point1.position.y;

            volume -= (p0x * p1y - p1x * p0y) / 2f;
        }

        return volume;
    }




    private class PointMass {
        public PointMass(Vector2 position, Softbody2 softbody) {
            this.position = position;
            this.softbody = softbody;
        }
        public Softbody2 softbody;

        public Vector2 position;
        public Vector2 previousPosition;
        public Vector2 velocity;
        public float mass { get { return softbody.mass / softbody.pointMassList.Count; } }
    }
    private class LinearConstraint {
        public Softbody2 softbody;

        public LinearConstraint(PointMass point0, PointMass point1, float inverseStiffness, bool volumetric, Softbody2 softbody) {
            this.point0 = point0;
            this.point1 = point1;
            defaultDistance = (point0.position - point1.position).magnitude;
            this.inverseStiffness = inverseStiffness;
            weirdA = inverseStiffness / Time.fixedDeltaTime / Time.fixedDeltaTime;
            this.volumetric = volumetric;
            this.softbody = softbody;
        }

        public PointMass point0;
        public PointMass point1;
        public float defaultDistance;
        public float inverseStiffness;
        public float weirdA;
        public float lambda;
        public float deltaLambda;
        public bool volumetric;

        public void Solve() {
            ComputeDeltaLambda(); // (18)
            if (volumetric) ComputeVolumeConstraint(); // Volume constraint
            ComputeDeltaDistance(); // (17)
        }
        public void ComputeDeltaLambda() { // (18)
            deltaLambda = (-((point0.position - point1.position).magnitude - defaultDistance) - weirdA * lambda) /
                (Mathf.Pow(point0.mass, -1) + Mathf.Pow(point1.mass, -1) + weirdA);
            lambda += deltaLambda;
        }
        public void ComputeVolumeConstraint() {
            int volumeConstraintToUse = 3;

            if (volumeConstraintToUse == 1) {
                // does NOT work
                float desiredPressure = softbody.GetVolume() - softbody.defasultVolume * softbody.pressure;

                List<Vector2> Jj = new List<Vector2>(new Vector2[softbody.pointMassList.Count]);
                for (int j = 0; j < softbody.linearConstraintsList.Count; j++) {
                    if (softbody.linearConstraintsList[j].volumetric == false) continue;
                    PointMass p0 = softbody.linearConstraintsList[j].point0;
                    PointMass p1 = softbody.linearConstraintsList[j].point1;
                    Jj[softbody.pointMassList.IndexOf(p0)] = -Vector2.Perpendicular(p0.position - p1.position) / 2f;
                    Jj[softbody.pointMassList.IndexOf(p1)] = Vector2.Perpendicular(p1.position - p0.position) / 2f;
                }
                Debug.DrawRay(point0.position, Jj[softbody.pointMassList.IndexOf(point0)], Color.red);
                Debug.DrawRay(point1.position, Jj[softbody.pointMassList.IndexOf(point1)], Color.blue);
                float Denom = weirdA;
                for (int i = 0; i < softbody.pointMassList.Count; i++) {
                    Denom += Mathf.Pow(softbody.pointMassList[i].mass, -1) * Vector2.Dot(Jj[i], Jj[i]);
                }

                float volumeDeviation = (-desiredPressure - weirdA * lambda) / Denom;

                for (int i = 0; i < softbody.pointMassList.Count; i++) {

                    Debug.DrawRay(point1.position, (Mathf.Pow(softbody.pointMassList[i].mass, -1) * volumeDeviation) * Jj[i] * 100, Color.black);
                    softbody.pointMassList[i].position += volumeDeviation * Mathf.Pow(softbody.pointMassList[i].mass, -1) * Jj[i];
                }

                lambda += volumeDeviation;
            }

            if (volumeConstraintToUse == 2) {
                float desiredPressure = softbody.GetVolume() - softbody.defasultVolume * softbody.pressure;

                float lenghOfAllConstraints = 0f;
                foreach (var constraint in softbody.linearConstraintsList) {
                    if (volumetric == false) continue;
                    lenghOfAllConstraints += (constraint.point0.position - constraint.point1.position).magnitude;
                }

                Vector2 forceToConstraint = desiredPressure * Vector2.Perpendicular(point0.position - point1.position).normalized * (point0.position - point1.position).magnitude / lenghOfAllConstraints;


                Debug.DrawRay(point0.position, forceToConstraint);
                Debug.DrawRay(point1.position, forceToConstraint);

                point0.position += forceToConstraint * Time.fixedDeltaTime;
                point1.position += forceToConstraint * Time.fixedDeltaTime;
            }

            if (volumeConstraintToUse == 3) {

                float desiredPressure = softbody.GetVolume() - softbody.defasultVolume * softbody.pressure;

                List<Vector2> Jj = new List<Vector2>(new Vector2[softbody.pointMassList.Count]);

                for (int j = 0; j < softbody.peaksList.Count; j++) {
                    if (softbody.linearConstraintsList[j].volumetric == false) continue;

                    float vipuclostb = (softbody.pointMassList[j].position - (softbody.peaksList[j].pointL.position + softbody.peaksList[j].pointR.position) / 2).magnitude;
                    Jj[j] = (
                        Vector2.Perpendicular(softbody.pointMassList[j].position - softbody.peaksList[j].pointL.position) + Vector2.Perpendicular(softbody.peaksList[j].pointR.position - softbody.pointMassList[j].position)
                        ) / 2f;

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

        }
        public void ComputeDeltaDistance() { // (17)
            point0.position += Mathf.Pow(point0.mass, -1) * (point0.position - point1.position).normalized * deltaLambda;
            point1.position -= Mathf.Pow(point1.mass, -1) * (point0.position - point1.position).normalized * deltaLambda;
        }
        public float GetConstraintValue() => (point0.position - point1.position).magnitude - defaultDistance;
        public void SetInverseStiffness(float inverseStiffness) {
            this.inverseStiffness = inverseStiffness;
            weirdA = inverseStiffness / Time.fixedDeltaTime / Time.fixedDeltaTime;
        }
    }
    private class Peak {
        public PointMass pointR;
        public PointMass pointL;
    }






    public void OnSceneGUI() {
        foreach (var point in mesh.vertices) {
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(point, 0.1f);
        }
    }

    [ExecuteInEditMode]
    private void OnDrawGizmos() {
        Gizmos.color = Color.green;



        foreach (var point in mesh.vertices) {
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(point, 0.05f);
        }


        foreach (var point in pointMassList) {
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(point.position, 0.05f);
        }


    }


#if UNITY_EDITOR
    [CustomEditor(typeof(Softbody2))]
    public class SoftbodyEditor : Editor {


        private void OnDrawGizmos() {
            Softbody2 pointMassSoftObject = (Softbody2)target;

            List<PointMass> pointMassList = pointMassSoftObject.pointMassList;
            foreach (PointMass point in pointMassList) {
                Gizmos.color = Color.green;
                Gizmos.DrawSphere(point.position, 0.1f);
            }


        }


        public void OnSceneGUI() {
            Softbody2 softBody = (Softbody2)target;

            List<PointMass> pointMassList = softBody.pointMassList;

            foreach (PointMass point in pointMassList) {
                EditorGUI.BeginChangeCheck();
                Vector2 position = Handles.PositionHandle(point.position, Quaternion.identity);
                if (EditorGUI.EndChangeCheck()) {
                    point.position = position;
                }
            }
        }

        public override void OnInspectorGUI() {
            base.OnInspectorGUI();
            Softbody2 softBody = (Softbody2)target;

            if (softBody.mass < 0) softBody.mass = 0;
            if (softBody.substeps < 1) softBody.substeps = 1;
            if (softBody.iterations < 1) softBody.iterations = 1;

        }


    }
#endif



}
