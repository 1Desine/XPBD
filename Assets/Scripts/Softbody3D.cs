#if UNITY_EDITOR
using UnityEditor;
#endif
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.Rendering;
using System.Xml.Linq;
using Unity.VisualScripting;
using System.IO;

public class Softbody3D : MonoBehaviour {

    [Header("there should be non of that")]
    [SerializeField] private List<PointMass> pointMassList;
    [SerializeField] private List<LinearConstraint> linearConstraintsList;
    private List<int> tornLinearConstraintsIndexesList = new List<int>();
    private List<LinearConstraint> linearMouseDragConstraintsList = new List<LinearConstraint>();

    [Header("Properties")]
    [SerializeField] private float mass = 1f;
    [SerializeField, Range(0f, 1f)] private float inverseStiffness = 0.5f;
    private float previousInverseStiffness;
    [SerializeField, Min(0.01f)] private float forceToTear = 0.2f;
    private bool torn = false;
    [SerializeField, Range(0f, 100f)] private float pressure = 1;
    private float defasultVolume;
    [SerializeField] private Vector3 gravity = Vector3.down * 9.8f;

    [Header("Computation")]
    [SerializeField] private uint substeps = 10; // splitting FixedUpdate
    [SerializeField] private uint iterations = 1; // of solving the constraints per substep

    [Header("Source of object")]
    [SerializeField] private string softbodyJsonRelativePath;
    [SerializeField] private Mesh mesh;

    private MeshFilter meshFilter;

    private void Awake() {
        meshFilter = GetComponent<MeshFilter>();

        SetSoftbodyFromMesh();

        //if (false) {
        //    linearConstraintsList.RemoveAt(4);
        //    linearConstraintsList.RemoveAt(3);
        //    linearConstraintsList.RemoveAt(1);
        //}
        //if (true) {
        //    linearConstraintsList[4].volumetric = false;
        //    linearConstraintsList[3].volumetric = false;
        //    linearConstraintsList[1].volumetric = false;
        //}

        //Debug.Log(points.Length);

        defasultVolume = GetVolume();
    }




    public float GetVolume() {
        float volume = 0.0f;
        for (int i = 0; i < linearConstraintsList.Count; i++) {
            if (linearConstraintsList[i].volumetric == false) continue;
            float p0x = linearConstraintsList[i].pointL.position.x;
            float p0y = linearConstraintsList[i].pointL.position.y;
            float p1x = linearConstraintsList[i].pointR.position.x;
            float p1y = linearConstraintsList[i].pointR.position.y;

            volume -= (p0x * p1y - p1x * p0y) / 2f;
        }

        return volume;
    }
    private void UpdameVisualMesh() {
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
    }
    private void Update() {
        //UpdameVisualMesh();

        //if (Input.GetMouseButton(0) && linearMouseDragConstraintsList.Count == 0) {
        //    Vector2 mouseWordPosition = Camera.main.ScreenToWorldPoint(Input.mousePosition);
        //    // Trying to find a triangle witch points are around the mouseWorldPosition
        //    for (int posInTriangles = 0; posInTriangles < mesh.triangles.Length; posInTriangles += 3) {
        //        float angle0 = Vector3.Dot(Vector3.Perpendicular(pointMassList[mesh.triangles[posInTriangles + 0]].position - pointMassList[mesh.triangles[posInTriangles + 2]].position), mouseWordPosition - pointMassList[mesh.triangles[posInTriangles + 2]].position);
        //        float angle1 = Vector3.Dot(Vector3.Perpendicular(pointMassList[mesh.triangles[posInTriangles + 1]].position - pointMassList[mesh.triangles[posInTriangles + 0]].position), mouseWordPosition - pointMassList[mesh.triangles[posInTriangles + 0]].position);
        //        float angle2 = Vector3.Dot(Vector3.Perpendicular(pointMassList[mesh.triangles[posInTriangles + 2]].position - pointMassList[mesh.triangles[posInTriangles + 1]].position), mouseWordPosition - pointMassList[mesh.triangles[posInTriangles + 1]].position);
        //        if (angle0 < 0 && angle1 < 0 && angle2 < 0 ||
        //            angle0 > 0 && angle1 > 0 && angle2 > 0) {
        //            linearMouseDragConstraintsList.Add(new LinearConstraint(new PointMass(mouseWordPosition, this), pointMassList[mesh.triangles[posInTriangles + 0]], 0.5f, false, this));
        //            linearMouseDragConstraintsList.Add(new LinearConstraint(new PointMass(mouseWordPosition, this), pointMassList[mesh.triangles[posInTriangles + 1]], 0.5f, false, this));
        //            linearMouseDragConstraintsList.Add(new LinearConstraint(new PointMass(mouseWordPosition, this), pointMassList[mesh.triangles[posInTriangles + 2]], 0.5f, false, this));
        //            break;
        //        }
        //    }
        //    foreach (LinearConstraint constraint in linearMouseDragConstraintsList) constraint.tearable = false;
        //}
        //if (Input.GetMouseButtonUp(0)) linearMouseDragConstraintsList.Clear();


        // VISUAL

        //points are connected clockwise
        foreach (LinearConstraint constraint in linearConstraintsList) {
            // constraint nornal
            //Debug.DrawRay((constraint.point0.position + constraint.point1.position) / 2f, Vector2.Perpendicular(constraint.point1.position - constraint.point0.position), Color.black);

            // Constraints perpendicular vectors
            //Debug.DrawRay(linearConstraintsList[i].point0.position, Vector2.Perpendicular(linearConstraintsList[i].point1.position - linearConstraintsList[i].point0.position).normalized);
            //Debug.DrawRay(linearConstraintsList[i].point1.position, Vector2.Perpendicular(linearConstraintsList[i].point1.position - linearConstraintsList[i].point0.position).normalized);
        }
        //floor formula y = x * angle
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
                    if (pointMass.position.y < -2) {
                        pointMass.position.y = -2;
                        pointMass.velocity.y = 0;
                    }
                    // radius bound
                    if (pointMass.position.magnitude > 5) {
                        pointMass.position -= pointMass.position * (pointMass.position.magnitude - 5);
                        pointMass.velocity = Vector3.zero;
                    }
                }

                // constraints
                foreach (LinearConstraint constraint in linearConstraintsList) constraint.Solve();
                // mouse dragging
                foreach (LinearConstraint constraint in linearMouseDragConstraintsList) {
                    constraint.pointL.position = Camera.main.ScreenToWorldPoint(Input.mousePosition);
                    constraint.pointL.velocity = Vector3.zero;
                    constraint.Solve();
                }
                if (tornLinearConstraintsIndexesList.Count != 0) {
                    foreach (int tornConstraint in tornLinearConstraintsIndexesList) {
                        linearConstraintsList.RemoveAt(tornConstraint);
                    }
                    tornLinearConstraintsIndexesList.Clear();
                }
            }

            // Moving points
            foreach (PointMass pointMass in pointMassList) {
                pointMass.velocity = (pointMass.position - pointMass.previousPosition) / deltaTime;

                // Friction - I don't get it, don't blame me
                if (pointMass.position.y < GetFloorY_AtX(pointMass.position.x)) {
                    Vector3 floorNormal = Quaternion.Euler(Vector3.forward * 90) * new Vector3(1, GetFloorY_AtX(1)).normalized;
                    //Debug.DrawRay(Vector2.up * GetFloorY_AtX(0), floorNormal, Color.red, 0.1f);

                    Vector3 velocityNormal = Vector2.Dot(floorNormal, pointMass.velocity) * floorNormal;
                    Vector3 velicityTangential = pointMass.velocity - velocityNormal;
                    Vector3 deltaVelocity = -Vector3.Min(Vector3.one,
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



    ///////////////////
    ///////////////////
    [Serializable]
    private class PointMass {
        public PointMass(Vector3 position, Softbody3D softbody) {
            this.position = position;
            this.softbody = softbody;
            id = softbody.pointMassList.Count;
        }
        public Softbody3D softbody;

        public int id;
        public Vector3 position;
        public Vector3 previousPosition;
        public Vector3 velocity;
        public float mass { get { return softbody.mass / softbody.pointMassList.Count; } }
    }
    ///////////////////
    ///////////////////
    [Serializable]
    private class LinearConstraint {
        public Softbody3D softbody;

        public LinearConstraint(PointMass pointL, PointMass pointR, float inverseStiffness, bool volumetric, Softbody3D softbody) {
            this.pointL = pointL;
            this.pointR = pointR;
            defaultDistance = (pointL.position - pointR.position).magnitude;
            this.inverseStiffness = inverseStiffness;
            weirdA = inverseStiffness / Time.fixedDeltaTime / Time.fixedDeltaTime;
            this.volumetric = volumetric;
            this.softbody = softbody;
        }

        public PointMass pointL;
        public PointMass pointR;
        public float defaultDistance;
        public float inverseStiffness;
        public float weirdA;
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
                softbody.tornLinearConstraintsIndexesList.Add(softbody.linearConstraintsList.IndexOf(this));
                softbody.tornLinearConstraintsIndexesList.Sort();
                softbody.tornLinearConstraintsIndexesList.Reverse();
                softbody.torn = true;
            }
        }
        public void ComputeVolumeConstraint() {
                float desiredPressure = softbody.GetVolume() - softbody.defasultVolume * softbody.pressure;

                List<Vector3> Jj = new List<Vector3>(new Vector3[softbody.pointMassList.Count]);

                //for (int j = 0; j < softbody.linearConstraintsList.Count; j++) {
                //    if (softbody.linearConstraintsList[j].volumetric == false) continue;
                //    Jj[softbody.linearConstraintsList[j].pointL.id] += Vector2.Perpendicular(softbody.linearConstraintsList[j].pointR.position - softbody.linearConstraintsList[j].pointL.position);
                //    Jj[softbody.linearConstraintsList[j].pointR.id] += Vector2.Perpendicular(softbody.linearConstraintsList[j].pointR.position - softbody.linearConstraintsList[j].pointL.position);
                //}

                for (int j = 0; j < softbody.linearConstraintsList.Count; j++) {
                    Debug.DrawRay(softbody.linearConstraintsList[j].pointL.position, Jj[softbody.linearConstraintsList[j].pointL.id] * desiredPressure, Color.red);
                    Debug.DrawRay(softbody.linearConstraintsList[j].pointR.position, Jj[softbody.linearConstraintsList[j].pointR.id] * desiredPressure, Color.red);
                }
                float Denom = weirdA;
                for (int i = 0; i < softbody.pointMassList.Count; i++) {
                    Denom += Mathf.Pow(softbody.pointMassList[i].mass, -1) * Vector3.Dot(Jj[i], Jj[i]);
                }

                float volumeDeviation = (-desiredPressure - weirdA * lambda) / Denom;

                for (int i = 0; i < softbody.pointMassList.Count; i++) {
                    softbody.pointMassList[i].position += volumeDeviation * Mathf.Pow(softbody.pointMassList[i].mass, -1) * Jj[i];
                }

                lambda += volumeDeviation;

        }
        public void ComputeDeltaDistance() { // (17)
            pointL.position += Mathf.Pow(pointL.mass, -1) * (pointL.position - pointR.position).normalized * deltaLambda;
            pointR.position -= Mathf.Pow(pointR.mass, -1) * (pointL.position - pointR.position).normalized * deltaLambda;
        }
        public float GetConstraintValue() => (pointL.position - pointR.position).magnitude - defaultDistance;
        public void SetInverseStiffness(float inverseStiffness) {
            this.inverseStiffness = inverseStiffness;
            weirdA = inverseStiffness / Time.fixedDeltaTime / Time.fixedDeltaTime;
        }
    }







    ///////////////////
    ///////////////////
    ///////////////////
    ///////////////////
    ///////////////////
    ///////////////////

    [ExecuteInEditMode]
    private void OnDrawGizmos() {
        Gizmos.color = Color.green;

        if (pointMassList != null) {
            foreach (PointMass point in pointMassList) {
                Gizmos.color = Color.green;
                Gizmos.DrawSphere(point.position, 0.05f);
            }
        }

        if (linearConstraintsList != null) {
            foreach (LinearConstraint linearConstraint in linearConstraintsList) {
                Gizmos.color = Color.green;
                Gizmos.DrawLine(linearConstraint.pointL.position, linearConstraint.pointR.position);
            }
        }


    }


#if UNITY_EDITOR
    [CustomEditor(typeof(Softbody3D))]
    public class SoftbodyEditor : Editor {
        public void OnSceneGUI() {
            Softbody3D softBody = (Softbody3D)target;

            List<PointMass> pointMassList = softBody.pointMassList;
            if (pointMassList != null) {
                foreach (PointMass point in pointMassList) {
                    EditorGUI.BeginChangeCheck();
                    Vector2 position = Handles.PositionHandle(point.position, Quaternion.identity);
                    if (EditorGUI.EndChangeCheck()) {
                        point.position = position;
                    }
                }
            }
        }

        public override void OnInspectorGUI() {
            base.OnInspectorGUI();
            Softbody3D softBody = (Softbody3D)target;

            if (softBody.mass < 0) softBody.mass = 0;
            if (softBody.substeps < 1) softBody.substeps = 1;
            if (softBody.iterations < 1) softBody.iterations = 1;

            GUILayout.Label("Load or save Softbody");
            if (GUILayout.Button("LoadFromMesh"))
                softBody.SetSoftbodyFromMesh();

            if (GUILayout.Button("LoadFromJson"))
                softBody.SetSoftbodyFromJson();

            if (GUILayout.Button("SaveToJson"))
                softBody.SaveSoftbodyToJson();
        }
    }
#endif




    // 
    // SAVE AND LOAD
    // 

    private class Softbody3DJsonData {
        public List<PointMass> pointMassList;
        public List<LinearConstraint> linearConstraintsList;

        public float mass = 1f;
        public float inverseStiffness = 0.5f;
        public float previousInverseStiffness;
        public Vector3 gravity = Vector3.down * 9.8f;
        public float pressure = 1;
        public float forceToTear = 1f;

        public uint substeps = 10;
        public uint iterations = 1;
    }



    public List<Particle> particlesList = new List<Particle>();
    public struct Particle {
        public Vector3 position;
    }
    public struct Constainer {
        public List<Particle> particlesList;
    }
    private void SaveSoftbodyToJson() {
        ///particlesList = new List<Particle>();
        particlesList.Add(new Particle { position = Vector3.left });

        Constainer container = new Constainer { particlesList = particlesList };
        Debug.Log(JsonUtility.ToJson(container));
        Debug.Log(JsonUtility.ToJson(particlesList[0]));

        SaveSystem.WriteJson(softbodyJsonRelativePath, JsonUtility.ToJson(particlesList));
        //SaveSystem.WriteJson(softbodyJsonRelativePath, JsonUtility.ToJson(this));
    }
    private void SetSoftbodyFromJson() {
        List<Particle> loadedParticlesList = JsonUtility.FromJson<List<Particle>>(SaveSystem.ReadJson(softbodyJsonRelativePath));

        foreach (Particle particle in loadedParticlesList) {
            Debug.Log(particle.position);
        }

        //Softbody3DJsonData loadedSoftbody = JsonUtility.FromJson<Softbody3DJsonData>(SaveSystem.ReadJson(softbodyJsonRelativePath));

        //pointMassList = loadedSoftbody.pointMassList;
        //linearConstraintsList = loadedSoftbody.linearConstraintsList;
        //
        //mass = loadedSoftbody.mass;
        //inverseStiffness = loadedSoftbody.inverseStiffness;
        //previousInverseStiffness = loadedSoftbody.previousInverseStiffness;
        //gravity = loadedSoftbody.gravity;
        //pressure = loadedSoftbody.pressure;
        //forceToTear = loadedSoftbody.forceToTear;
        //
        //substeps = loadedSoftbody.substeps;
        //iterations = loadedSoftbody.iterations;
    }

    private void SetSoftbodyFromMesh() {
        pointMassList = new List<PointMass>();
        linearConstraintsList = new List<LinearConstraint>();

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
                if (pointMass.position == mesh.vertices[mesh.triangles[posInTriangles + 0]]) v0 = pointMass;
                if (pointMass.position == mesh.vertices[mesh.triangles[posInTriangles + 1]]) v1 = pointMass;
                if (pointMass.position == mesh.vertices[mesh.triangles[posInTriangles + 2]]) v2 = pointMass;
            }

            // Check from pointMass, does it have connections with other pointMasses in this triangle
            if (PointsAreConnected(v0, v1) == false) linearConstraintsList.Add(new LinearConstraint(v0, v1, inverseStiffness, true, this));
            if (PointsAreConnected(v1, v2) == false) linearConstraintsList.Add(new LinearConstraint(v1, v2, inverseStiffness, true, this));
            if (PointsAreConnected(v2, v0) == false) linearConstraintsList.Add(new LinearConstraint(v2, v0, inverseStiffness, true, this));
        }
    }
    private bool PointsAreConnected(PointMass point0, PointMass point1) {
        foreach (var constraint in linearConstraintsList) {
            if (constraint.pointL == point0 && constraint.pointR == point1
             || constraint.pointL == point1 && constraint.pointR == point0) return true;
        }
        return false;
    }


}
