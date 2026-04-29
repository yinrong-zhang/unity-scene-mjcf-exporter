using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using System.Xml;
using Unity.Robotics.UrdfImporter;
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class UnitySceneMjcfExporter
{
    const string MenuPath = "Gewu/MuJoCo/Export Current Unity Scene To MJCF XML";
    const string SelectedMenuPath = "Gewu/MuJoCo/Export Selected GameObjects To MJCF XML";
    const float ThinMeshThickness = 1e-4f;

    static readonly CultureInfo Invariant = CultureInfo.InvariantCulture;

    [MenuItem(MenuPath)]
    static void ExportCurrentScene()
    {
        var scene = SceneManager.GetActiveScene();
        if (!scene.IsValid())
        {
            Debug.LogWarning("No valid active scene to export.");
            return;
        }

        var path = EditorUtility.SaveFilePanel("Export Unity Scene to MJCF XML", DefaultExportDirectory(), scene.name, "xml");
        if (string.IsNullOrEmpty(path))
            return;

        try
        {
            var exporter = new Exporter(scene);
            exporter.Export(path, null);
            Debug.Log($"MJCF XML exported to {path}\n{exporter.Report}");
        }
        catch (Exception ex)
        {
            Debug.LogError($"Failed to export MJCF XML: {ex}");
        }
    }

    [MenuItem(SelectedMenuPath)]
    static void ExportSelectedGameObjects()
    {
        var roots = Selection.gameObjects;
        if (roots == null || roots.Length == 0)
        {
            Debug.LogWarning("Select one or more robot root GameObjects before exporting.");
            return;
        }

        var scene = SceneManager.GetActiveScene();
        if (!scene.IsValid())
        {
            Debug.LogWarning("No valid active scene to export.");
            return;
        }

        var path = EditorUtility.SaveFilePanel("Export Selected GameObjects to MJCF XML", DefaultExportDirectory(), scene.name + "_selected", "xml");
        if (string.IsNullOrEmpty(path))
            return;

        try
        {
            var exporter = new Exporter(scene);
            exporter.Export(path, roots);
            Debug.Log($"Selected MJCF XML exported to {path}\n{exporter.Report}");
        }
        catch (Exception ex)
        {
            Debug.LogError($"Failed to export selected MJCF XML: {ex}");
        }
    }

    [MenuItem(SelectedMenuPath, true)]
    static bool CanExportSelectedGameObjects()
    {
        return Selection.gameObjects != null && Selection.gameObjects.Length > 0;
    }

    static string DefaultExportDirectory()
    {
        return Path.GetDirectoryName(Application.dataPath) ?? Application.dataPath;
    }

    sealed class Exporter
    {
        readonly Scene _scene;
        readonly XmlDocument _doc = new XmlDocument();
        readonly Dictionary<string, string> _meshNames = new Dictionary<string, string>();
        readonly Dictionary<string, string> _fallbackMeshAssets = new Dictionary<string, string>();
        readonly Dictionary<Material, string> _materialNames = new Dictionary<Material, string>();
        readonly HashSet<string> _usedNames = new HashSet<string>();
        XmlElement _asset;
        XmlElement _worldbody;
        string _meshDirectory;
        string _meshDirectoryName;

        int _bodyCount;
        int _geomCount;
        int _jointCount;
        int _meshCount;
        int _materialCount;
        int _cameraCount;
        int _lightCount;

        public string Report =>
            $"Bodies: {_bodyCount}, Geoms: {_geomCount}, Joints: {_jointCount}, Meshes: {_meshCount}, Materials: {_materialCount}, Cameras: {_cameraCount}, Lights: {_lightCount}";

        public Exporter(Scene scene)
        {
            _scene = scene;
        }

        public void Export(string path, GameObject[] selectedRoots)
        {
            var outputDirectory = Path.GetDirectoryName(path);
            var modelName = Path.GetFileNameWithoutExtension(path);
            _meshDirectoryName = SafeName(modelName + "_meshes");
            _meshDirectory = Path.Combine(outputDirectory ?? string.Empty, _meshDirectoryName);
            Directory.CreateDirectory(_meshDirectory);

            _doc.AppendChild(_doc.CreateXmlDeclaration("1.0", "utf-8", null));

            var mujoco = Element("mujoco");
            mujoco.SetAttribute("model", SafeName(string.IsNullOrEmpty(_scene.name) ? "unity_scene" : _scene.name));
            _doc.AppendChild(mujoco);

            var compiler = Append(mujoco, "compiler");
            compiler.SetAttribute("angle", "radian");
            compiler.SetAttribute("meshdir", _meshDirectoryName);
            compiler.SetAttribute("balanceinertia", "true");

            var option = Append(mujoco, "option");
            option.SetAttribute("gravity", Vec(Mj(Physics.gravity)));

            _asset = Append(mujoco, "asset");
            _worldbody = Append(mujoco, "worldbody");

            var roots = selectedRoots == null || selectedRoots.Length == 0 ? _scene.GetRootGameObjects() : SelectedTopLevelRoots(selectedRoots);
            foreach (var root in roots)
            {
                if (ShouldSkip(root))
                    continue;

                if (ContainsRobotBoundary(root.transform))
                    ExportTransform(root.transform, _worldbody, true);
                else
                    ExportStaticSceneMeshes(root.transform);
            }

            // 验证和修复导出的MJCF
            ValidateAndFixMJCF(mujoco);

            using (var stream = File.Open(path, FileMode.Create))
            using (var writer = new XmlTextWriter(stream, new UTF8Encoding(false)))
            {
                writer.Formatting = Formatting.Indented;
                _doc.WriteTo(writer);
            }
        }

        void ValidateAndFixMJCF(XmlElement mujoco)
        {
            // 1. 确保有默认材质
            var asset = mujoco.SelectSingleNode("asset") as XmlElement;
            if (asset == null)
                return;

            var defaultMat = asset.SelectSingleNode("material[@name='default_material']");
            if (defaultMat == null)
            {
                var matElement = _doc.CreateElement("material");
                matElement.SetAttribute("name", "default_material");
                matElement.SetAttribute("rgba", "0.9 0.9 0.9 1");
                asset.AppendChild(matElement);
            }

            // 2. 修复所有透明度为0的材质
            foreach (XmlElement mat in asset.SelectNodes("material"))
            {
                var rgba = mat.GetAttribute("rgba");
                if (!string.IsNullOrEmpty(rgba))
                {
                    var parts = rgba.Split(' ');
                    if (parts.Length == 4 && float.TryParse(parts[3], NumberStyles.Any, Invariant, out var alpha))
                    {
                        if (alpha <= 0)
                        {
                            parts[3] = "1";
                            mat.SetAttribute("rgba", string.Join(" ", parts));
                        }
                    }
                }
            }

            // 3. 为所有没有material的mesh geom添加默认材质
            var worldbody = mujoco.SelectSingleNode("worldbody") as XmlElement;
            if (worldbody != null)
            {
                foreach (XmlElement geom in worldbody.SelectNodes(".//geom[@type='mesh'][not(@material)]"))
                {
                    geom.SetAttribute("material", "default_material");
                }
            }

            // 4. 如果没有light，添加一个默认方向光
            var lights = mujoco.SelectNodes("worldbody/light");
            if (lights.Count == 0 && worldbody != null)
            {
                var dirLight = _doc.CreateElement("light");
                dirLight.SetAttribute("name", UniqueName("default_directional_light"));
                dirLight.SetAttribute("pos", "0 0 5");
                dirLight.SetAttribute("dir", "0 0 -1");
                dirLight.SetAttribute("diffuse", "1 1 1");
                worldbody.AppendChild(dirLight);
            }
        }

        XmlElement ExportTransform(Transform transform, XmlElement parent, bool isRoot)
        {
            var body = Append(parent, "body");
            body.SetAttribute("name", UniqueName(transform.name));
            body.SetAttribute("pos", Vec(Mj(ExportBodyPosition(transform, isRoot))));
            body.SetAttribute("quat", Quat(Mj(isRoot ? transform.rotation : transform.localRotation)));
            _bodyCount++;

            ExportInertial(transform, body);
            ExportJoint(transform, body);
            ExportColliders(transform, body);

            var visualGeomCountBefore = _geomCount;
            ExportMeshRenderer(transform, body);
            if (CanOwnDescendantVisualMeshes(transform))
                ExportDescendantVisualMeshes(transform, body);
            
            // 如果还是没有geom被添加，尝试导出所有的mesh
            if (_geomCount == visualGeomCountBefore)
            {
                ExportAllMeshFilters(transform, body);
            }
            
            if (_geomCount == visualGeomCountBefore)
                ExportFallbackRobotMesh(transform, body);

            ExportCamera(transform, body);
            ExportLight(transform, body);

            for (var i = 0; i < transform.childCount; i++)
            {
                var child = transform.GetChild(i);
                if (ShouldSkip(child.gameObject) || ShouldSkipChildBody(transform, child))
                    continue;

                if (ContainsRobotBoundary(child))
                    ExportTransform(child, body, false);
                else
                    ExportStaticSceneMeshes(child);
            }

            return body;
        }

        void ExportStaticSceneMeshes(Transform root)
        {
            foreach (var meshFilter in root.GetComponentsInChildren<MeshFilter>(true))
            {
                if (meshFilter.sharedMesh == null || ShouldSkip(meshFilter.gameObject))
                    continue;

                var renderer = meshFilter.GetComponent<MeshRenderer>();
                ExportBakedMeshGeom(
                    meshFilter.sharedMesh,
                    renderer != null ? renderer.sharedMaterial : null,
                    _worldbody,
                    meshFilter.transform.localToWorldMatrix,
                    meshFilter.name,
                    visualOnly: false);
            }
        }


        static Vector3 ExportBodyPosition(Transform transform, bool isRoot)
        {
            if (isRoot || transform.parent == null)
                return transform.position;

            // MuJoCo bodies do not inherit scale. Recompute the child offset from
            // Unity world positions so parent scale is baked into body spacing.
            return Quaternion.Inverse(transform.parent.rotation) * (transform.position - transform.parent.position);
        }

        static Matrix4x4 ExportBodyWorldToLocalMatrix(Transform transform)
        {
            return Matrix4x4.TRS(transform.position, transform.rotation, Vector3.one).inverse;
        }

        void ExportInertial(Transform transform, XmlElement body)
        {
            var articulation = transform.GetComponent<ArticulationBody>();
            var rigidbody = transform.GetComponent<Rigidbody>();
            var urdfInertial = transform.GetComponent<UrdfInertial>();

            var mass = articulation != null ? articulation.mass : rigidbody != null ? rigidbody.mass : 0f;
            if (mass <= 0f && urdfInertial == null)
                return;

            var inertial = Append(body, "inertial");
            inertial.SetAttribute("mass", Number(Mathf.Max(mass, 1e-6f)));

            var centerOfMass = articulation != null ? articulation.centerOfMass : rigidbody != null ? rigidbody.centerOfMass : urdfInertial != null ? urdfInertial.centerOfMass : Vector3.zero;
            centerOfMass = Vector3.Scale(centerOfMass, Abs(transform.lossyScale));
            inertial.SetAttribute("pos", Vec(Mj(centerOfMass)));

            var inertia = articulation != null ? articulation.inertiaTensor : rigidbody != null ? rigidbody.inertiaTensor : urdfInertial != null ? urdfInertial.inertiaTensor : Vector3.one * 1e-6f;
            inertial.SetAttribute("diaginertia", Vec(MjInertia(inertia, transform.lossyScale)));
        }

        void ExportJoint(Transform transform, XmlElement body)
        {
            var articulation = transform.GetComponent<ArticulationBody>();
            var urdfJoint = transform.GetComponent<UrdfJoint>();

            if (urdfJoint is UrdfJointFixed || articulation == null && urdfJoint == null)
                return;

            var type = JointType(articulation, urdfJoint);
            if (string.IsNullOrEmpty(type))
                return;

            var joint = Append(body, "joint");
            joint.SetAttribute("name", UniqueName(!string.IsNullOrEmpty(urdfJoint?.jointName) ? urdfJoint.jointName : transform.name + "_joint"));
            joint.SetAttribute("type", type);
            if (type == "hinge" || type == "slide")
                joint.SetAttribute("axis", Vec(JointAxis(articulation)));

            if (articulation != null)
            {
                joint.SetAttribute("damping", Number(Mathf.Max(articulation.angularDamping, articulation.linearDamping)));
                joint.SetAttribute("frictionloss", Number(Mathf.Max(0f, articulation.jointFriction)));

                if (type == "hinge" || type == "slide")
                {
                    var drive = articulation.xDrive;
                    if (drive.lowerLimit < drive.upperLimit)
                    {
                        var lower = type == "hinge" ? drive.lowerLimit * Mathf.Deg2Rad : drive.lowerLimit;
                        var upper = type == "hinge" ? drive.upperLimit * Mathf.Deg2Rad : drive.upperLimit;
                        joint.SetAttribute("range", $"{Number(lower)} {Number(upper)}");
                        joint.SetAttribute("limited", "true");
                    }
                }
            }

            _jointCount++;
        }

        void ExportColliders(Transform transform, XmlElement body)
        {
            foreach (var collider in transform.GetComponents<Collider>())
            {
                if (!collider.enabled)
                    continue;

                switch (collider)
                {
                    case BoxCollider box:
                        ExportBoxCollider(box, body);
                        break;
                    case SphereCollider sphere:
                        ExportSphereCollider(sphere, body);
                        break;
                    case CapsuleCollider capsule:
                        ExportCapsuleCollider(capsule, body);
                        break;
                    case MeshCollider meshCollider:
                        if (meshCollider.sharedMesh != null)
                            ExportMeshGeom(meshCollider.sharedMesh, null, body, Vector3.zero, Quaternion.identity, meshCollider.transform.lossyScale, false);
                        break;
                }
            }
        }

        void ExportMeshRenderer(Transform transform, XmlElement body)
        {
            var meshFilter = transform.GetComponent<MeshFilter>();
            var renderer = transform.GetComponent<MeshRenderer>();
            if (meshFilter == null || meshFilter.sharedMesh == null)
                return;

            var collider = transform.GetComponent<Collider>();
            if (collider is MeshCollider)
                return;

            // 确保导出所有的mesh，带有渲染器或没有都要导出
            ExportMeshGeom(meshFilter.sharedMesh, renderer != null ? renderer.sharedMaterial : null, body, Vector3.zero, Quaternion.identity, transform.lossyScale, visualOnly: false);
        }

        void ExportDescendantVisualMeshes(Transform link, XmlElement body)
        {
            foreach (var meshFilter in link.GetComponentsInChildren<MeshFilter>(true))
            {
                if (meshFilter.transform == link || meshFilter.sharedMesh == null)
                    continue;

                if (!BelongsToCurrentLink(link, meshFilter.transform))
                    continue;

                var renderer = meshFilter.GetComponent<MeshRenderer>();
                var localMatrix = ExportBodyWorldToLocalMatrix(link) * meshFilter.transform.localToWorldMatrix;
                // 确保所有descendant mesh都被导出，包括没有renderer的
                ExportBakedMeshGeom(meshFilter.sharedMesh, renderer != null ? renderer.sharedMaterial : null, body, localMatrix, meshFilter.name, visualOnly: false);
            }
        }

        void ExportFallbackRobotMesh(Transform transform, XmlElement body)
        {
            var meshKey = FallbackMeshKey(transform.name);
            if (string.IsNullOrEmpty(meshKey))
                return;

            var assetPath = FindFallbackMeshAssetPath(meshKey);
            if (string.IsNullOrEmpty(assetPath))
                return;

            ExportModelAssetGeom(assetPath, body);
        }

        void ExportAllMeshFilters(Transform transform, XmlElement body)
        {
            // 查找所有直接或间接的MeshFilter并导出为OBJ
            var meshFilters = transform.GetComponents<MeshFilter>();
            foreach (var meshFilter in meshFilters)
            {
                if (meshFilter.sharedMesh == null)
                    continue;

                var renderer = meshFilter.GetComponent<MeshRenderer>();
                ExportMeshGeom(meshFilter.sharedMesh, renderer != null ? renderer.sharedMaterial : null, body, Vector3.zero, Quaternion.identity, Vector3.one, visualOnly: false);
            }
        }

        void ExportBoxCollider(BoxCollider collider, XmlElement body)
        {
            var scale = Abs(collider.transform.lossyScale);
            var size = Vector3.Scale(collider.size, scale) * 0.5f;
            var geom = NewGeom(body, collider.name, "box");
            geom.SetAttribute("size", Vec(MjSize(size)));
            geom.SetAttribute("pos", Vec(Mj(Vector3.Scale(collider.center, scale))));
        }

        void ExportSphereCollider(SphereCollider collider, XmlElement body)
        {
            var scale = Abs(collider.transform.lossyScale);
            var radius = collider.radius * Mathf.Max(scale.x, scale.y, scale.z);
            var geom = NewGeom(body, collider.name, "sphere");
            geom.SetAttribute("size", Number(radius));
            geom.SetAttribute("pos", Vec(Mj(Vector3.Scale(collider.center, scale))));
        }

        void ExportCapsuleCollider(CapsuleCollider collider, XmlElement body)
        {
            var scale = Abs(collider.transform.lossyScale);
            var axisScale = collider.direction == 0 ? scale.x : collider.direction == 1 ? scale.y : scale.z;
            var radiusScale = collider.direction == 0 ? Mathf.Max(scale.y, scale.z) : collider.direction == 1 ? Mathf.Max(scale.x, scale.z) : Mathf.Max(scale.x, scale.y);
            var radius = collider.radius * radiusScale;
            var halfLength = Mathf.Max(0f, collider.height * axisScale * 0.5f - radius);
            var fromTo = CapsuleFromTo(collider.direction, halfLength);
            var center = Vector3.Scale(collider.center, scale);

            var geom = NewGeom(body, collider.name, "capsule");
            geom.SetAttribute("size", Number(radius));
            geom.SetAttribute("fromto", $"{Vec(Mj(fromTo.Item1 + center))} {Vec(Mj(fromTo.Item2 + center))}");
        }

        void ExportMeshGeom(Mesh mesh, Material material, XmlElement body, Vector3 localPosition, Quaternion localRotation, Vector3 localScale, bool visualOnly = true)
        {
            var geom = NewGeom(body, mesh.name, "mesh");
            geom.SetAttribute("mesh", MeshName(mesh, localScale));
            geom.SetAttribute("pos", Vec(Mj(localPosition)));
            geom.SetAttribute("quat", Quat(Mj(localRotation)));
            if (visualOnly)
                SetVisualOnly(geom);

            if (material != null)
                geom.SetAttribute("material", MaterialName(material));

        }

        void ExportBakedMeshGeom(Mesh mesh, Material material, XmlElement body, Matrix4x4 localMatrix, string baseName, bool visualOnly = true)
        {
            var geom = NewGeom(body, baseName, "mesh");
            geom.SetAttribute("mesh", BakedMeshName(mesh, localMatrix, baseName));
            geom.SetAttribute("pos", "0 0 0");
            geom.SetAttribute("quat", Quat(Mj(Quaternion.identity)));
            if (visualOnly)
                SetVisualOnly(geom);

            if (material != null)
                geom.SetAttribute("material", MaterialName(material));
        }

        void ExportModelAssetGeom(string assetPath, XmlElement body)
        {
            var geom = NewGeom(body, Path.GetFileNameWithoutExtension(assetPath), "mesh");
            geom.SetAttribute("mesh", ModelAssetMeshName(assetPath));
            geom.SetAttribute("pos", "0 0 0");
            geom.SetAttribute("quat", Quat(Mj(Quaternion.identity)));
            SetVisualOnly(geom);
        }

        void ExportCamera(Transform transform, XmlElement body)
        {
            var camera = transform.GetComponent<Camera>();
            if (camera == null || !camera.enabled)
                return;

            var element = Append(body, "camera");
            element.SetAttribute("name", UniqueName(camera.name));
            element.SetAttribute("pos", "0 0 0");
            element.SetAttribute("quat", Quat(Mj(Quaternion.identity)));
            element.SetAttribute("fovy", Number(camera.fieldOfView * Mathf.Deg2Rad));
            _cameraCount++;
        }

        void ExportLight(Transform transform, XmlElement body)
        {
            var light = transform.GetComponent<Light>();
            if (light == null || !light.enabled)
                return;

            var element = Append(body, "light");
            element.SetAttribute("name", UniqueName(light.name));
            // 使用实际的transform位置而不是固定的0,0,0
            element.SetAttribute("pos", Vec(Mj(transform.position)));
            element.SetAttribute("dir", Vec(Mj(transform.forward)));
            element.SetAttribute("diffuse", $"{Number(light.color.r)} {Number(light.color.g)} {Number(light.color.b)}");
            _lightCount++;
        }

        XmlElement NewGeom(XmlElement body, string baseName, string type)
        {
            var geom = Append(body, "geom");
            geom.SetAttribute("name", UniqueName(baseName + "_geom"));
            geom.SetAttribute("type", type);
            _geomCount++;
            return geom;
        }

        static void SetVisualOnly(XmlElement geom)
        {
            geom.SetAttribute("contype", "0");
            geom.SetAttribute("conaffinity", "0");
            geom.SetAttribute("group", "2");
        }

        string MeshName(Mesh mesh, Vector3 scale)
        {
            var safeScale = Abs(scale);
            var key = $"{mesh.GetInstanceID()}:{Number(safeScale.x)}:{Number(safeScale.y)}:{Number(safeScale.z)}";
            if (_meshNames.TryGetValue(key, out var existing))
                return existing;

            var name = UniqueName(mesh.name + "_mesh");
            _meshNames[key] = name;

            var meshElement = Append(_asset, "mesh");
            meshElement.SetAttribute("name", name);
            meshElement.SetAttribute("file", name + ".obj");
            WriteObjMesh(Path.Combine(_meshDirectory, name + ".obj"), mesh, safeScale);
            _meshCount++;
            return name;
        }

        string BakedMeshName(Mesh mesh, Matrix4x4 localMatrix, string baseName)
        {
            var name = UniqueName(baseName + "_mesh");

            var meshElement = Append(_asset, "mesh");
            meshElement.SetAttribute("name", name);
            meshElement.SetAttribute("file", name + ".obj");
            WriteObjMesh(Path.Combine(_meshDirectory, name + ".obj"), mesh, localMatrix);
            _meshCount++;
            return name;
        }

        string MaterialName(Material material)
        {
            if (_materialNames.TryGetValue(material, out var existing))
                return existing;

            var name = UniqueName(material.name + "_mat");
            _materialNames[material] = name;

            var element = Append(_asset, "material");
            element.SetAttribute("name", name);
            var color = material.HasProperty("_Color") ? material.color : Color.white;
            // 强制alpha为1，确保材质完全不透明
            element.SetAttribute("rgba", $"{Number(color.r)} {Number(color.g)} {Number(color.b)} 1");
            _materialCount++;
            return name;
        }

        string ModelAssetMeshName(string assetPath)
        {
            if (_fallbackMeshAssets.TryGetValue(assetPath, out var existing))
                return existing;

            var name = UniqueName(Path.GetFileNameWithoutExtension(assetPath) + "_mesh");
            _fallbackMeshAssets[assetPath] = name;

            var meshElement = Append(_asset, "mesh");
            meshElement.SetAttribute("name", name);
            meshElement.SetAttribute("file", name + ".obj");
            WriteModelAssetObj(Path.Combine(_meshDirectory, name + ".obj"), assetPath);
            _meshCount++;
            return name;
        }

        static string FindFallbackMeshAssetPath(string meshKey)
        {
            var preferredPaths = new[]
            {
                $"Assets/urdf/go2_description/meshes/{meshKey}.dae",
                $"Assets/urdf/go2_description/dae/{meshKey}.dae",
                $"Assets/urdf/go2_description/meshes/{meshKey}.stl",
                $"Assets/urdf/go2_description/dae/{meshKey}.stl",
                $"Assets/urdf/go2w_description/dae/{meshKey}.dae",
                $"Assets/urdf/go2w_description/dae/{meshKey}.stl"
            };

            foreach (var path in preferredPaths)
            {
                if (File.Exists(path))
                    return path;
            }

            return null;
        }

        static string FallbackMeshKey(string objectName)
        {
            var name = objectName.ToLowerInvariant();
            var mirrored = name.StartsWith("fl_") || name.StartsWith("rl_");

            if (name == "base" || name == "trunk")
                return File.Exists("Assets/urdf/go2_description/meshes/trunk.dae") ? "trunk" : "base";
            if (name.EndsWith("_hip") || name == "hip")
                return "hip";
            if (name.EndsWith("_thigh") || name == "thigh")
                return mirrored && File.Exists("Assets/urdf/go2_description/meshes/thigh_mirror.dae") ? "thigh_mirror" : "thigh";
            if (name.EndsWith("_calf") || name == "calf")
                return mirrored && File.Exists("Assets/urdf/go2_description/meshes/calf_mirror.dae") ? "calf_mirror" : "calf";
            if (name.EndsWith("_foot") || name == "foot")
                return "foot";

            return null;
        }

        static void WriteObjMesh(string path, Mesh mesh, Vector3 scale)
        {
            var builder = new StringBuilder();
            builder.AppendLine("# Exported from Unity scene to MJCF companion OBJ");

            var vertices = new List<Vector3>(mesh.vertexCount);
            foreach (var vertex in mesh.vertices)
                vertices.Add(Mj(Vector3.Scale(vertex, scale)));

            WriteObjGeometry(builder, vertices, mesh.triangles, 0);

            File.WriteAllText(path, builder.ToString(), new UTF8Encoding(false));
        }

        static void WriteObjMesh(string path, Mesh mesh, Matrix4x4 localMatrix)
        {
            var builder = new StringBuilder();
            builder.AppendLine("# Exported from Unity scene visual mesh to MJCF companion OBJ");

            var vertices = new List<Vector3>(mesh.vertexCount);
            foreach (var vertex in mesh.vertices)
                vertices.Add(Mj(localMatrix.MultiplyPoint3x4(vertex)));

            WriteObjGeometry(builder, vertices, mesh.triangles, 0);

            File.WriteAllText(path, builder.ToString(), new UTF8Encoding(false));
        }

        static void WriteModelAssetObj(string path, string assetPath)
        {
            var root = AssetDatabase.LoadAssetAtPath<GameObject>(assetPath);
            if (root == null)
            {
                var mesh = AssetDatabase.LoadAssetAtPath<Mesh>(assetPath);
                if (mesh != null)
                {
                    WriteObjMesh(path, mesh, Vector3.one);
                    return;
                }

                throw new InvalidOperationException($"Could not load mesh asset at {assetPath}");
            }

            var builder = new StringBuilder();
            builder.AppendLine("# Exported from Unity imported model hierarchy to MJCF companion OBJ");
            var vertexOffset = 0;
            var rootMatrix = root.transform.worldToLocalMatrix;

            foreach (var meshFilter in root.GetComponentsInChildren<MeshFilter>(true))
            {
                var mesh = meshFilter.sharedMesh;
                if (mesh == null)
                    continue;

                var matrix = rootMatrix * meshFilter.transform.localToWorldMatrix;
                var vertices = new List<Vector3>(mesh.vertexCount);
                foreach (var vertex in mesh.vertices)
                    vertices.Add(Mj(matrix.MultiplyPoint3x4(vertex)));

                vertexOffset += WriteObjGeometry(builder, vertices, mesh.triangles, vertexOffset);
            }

            if (vertexOffset == 0)
                throw new InvalidOperationException($"No MeshFilter geometry found in model asset {assetPath}");

            File.WriteAllText(path, builder.ToString(), new UTF8Encoding(false));
        }

        static int WriteObjGeometry(StringBuilder builder, List<Vector3> vertices, int[] triangles, int vertexOffset)
        {
            var thinAxis = ThinAxis(vertices);
            var writtenVertexCount = thinAxis < 0 ? vertices.Count : vertices.Count * 2;

            if (thinAxis < 0)
            {
                foreach (var vertex in vertices)
                    WriteObjVertex(builder, vertex);
            }
            else
            {
                var offset = Vector3.zero;
                offset[thinAxis] = ThinMeshThickness * 0.5f;

                foreach (var vertex in vertices)
                    WriteObjVertex(builder, vertex + offset);
                foreach (var vertex in vertices)
                    WriteObjVertex(builder, vertex - offset);
            }

            for (var i = 0; i + 2 < triangles.Length; i += 3)
            {
                WriteObjFace(builder, vertexOffset + triangles[i] + 1, vertexOffset + triangles[i + 1] + 1, vertexOffset + triangles[i + 2] + 1);
            }

            if (thinAxis >= 0)
            {
                var bottomOffset = vertexOffset + vertices.Count;
                for (var i = 0; i + 2 < triangles.Length; i += 3)
                {
                    WriteObjFace(builder, bottomOffset + triangles[i + 2] + 1, bottomOffset + triangles[i + 1] + 1, bottomOffset + triangles[i] + 1);
                }
            }

            return writtenVertexCount;
        }

        static int ThinAxis(List<Vector3> vertices)
        {
            if (vertices.Count < 3)
                return -1;

            var min = vertices[0];
            var max = vertices[0];
            for (var i = 1; i < vertices.Count; i++)
            {
                min = Vector3.Min(min, vertices[i]);
                max = Vector3.Max(max, vertices[i]);
            }

            var extents = max - min;
            var maxExtent = Mathf.Max(extents.x, extents.y, extents.z);
            if (maxExtent < 1e-6f)
                return -1;

            var minExtent = Mathf.Min(extents.x, extents.y, extents.z);
            if (minExtent > 1e-6f)
                return -1;

            if (extents.x <= extents.y && extents.x <= extents.z)
                return 0;
            if (extents.y <= extents.x && extents.y <= extents.z)
                return 1;

            return 2;
        }

        static void WriteObjVertex(StringBuilder builder, Vector3 vertex)
        {
            builder.Append("v ")
                .Append(Vec(vertex))
                .AppendLine();
        }

        static void WriteObjFace(StringBuilder builder, int a, int b, int c)
        {
            builder.Append("f ")
                .Append(a).Append(' ')
                .Append(b).Append(' ')
                .Append(c)
                .AppendLine();
        }

        static string JointType(ArticulationBody articulation, UrdfJoint urdfJoint)
        {
            if (urdfJoint is UrdfJointRevolute || urdfJoint is UrdfJointContinuous)
                return "hinge";
            if (urdfJoint is UrdfJointPrismatic)
                return "slide";
            if (urdfJoint is UrdfJointFloating)
                return "free";

            if (articulation == null)
                return null;

            switch (articulation.jointType)
            {
                case ArticulationJointType.RevoluteJoint:
                    return "hinge";
                case ArticulationJointType.PrismaticJoint:
                    return "slide";
                case ArticulationJointType.FixedJoint:
                    return null;
                case ArticulationJointType.SphericalJoint:
                    return "ball";
                default:
                    return null;
            }
        }

        static Vector3 JointAxis(ArticulationBody articulation)
        {
            if (articulation == null)
                return new Vector3(1, 0, 0);

            return Mj(articulation.anchorRotation * Vector3.right).normalized;
        }

        static Tuple<Vector3, Vector3> CapsuleFromTo(int direction, float halfLength)
        {
            var axis = direction == 0 ? Vector3.right : direction == 1 ? Vector3.up : Vector3.forward;
            return Tuple.Create(-axis * halfLength, axis * halfLength);
        }

        static bool ShouldSkip(GameObject gameObject)
        {
            return gameObject.hideFlags == HideFlags.NotEditable || gameObject.hideFlags == HideFlags.HideAndDontSave;
        }

        static GameObject[] SelectedTopLevelRoots(GameObject[] selection)
        {
            var roots = new List<GameObject>();
            foreach (var gameObject in selection)
            {
                if (gameObject == null)
                    continue;

                var hasSelectedAncestor = false;
                var parent = gameObject.transform.parent;
                while (parent != null)
                {
                    foreach (var selected in selection)
                    {
                        if (selected != null && selected.transform == parent)
                        {
                            hasSelectedAncestor = true;
                            break;
                        }
                    }

                    if (hasSelectedAncestor)
                        break;

                    parent = parent.parent;
                }

                if (!hasSelectedAncestor && !roots.Contains(gameObject))
                    roots.Add(gameObject);
            }

            return roots.ToArray();
        }

        static bool ShouldSkipChildBody(Transform parent, Transform child)
        {
            if (!HasRobotBoundaryComponent(parent))
                return false;

            return string.Equals(child.name, "Visuals", StringComparison.OrdinalIgnoreCase)
                || string.Equals(child.name, "Collisions", StringComparison.OrdinalIgnoreCase);
        }

        static bool CanOwnDescendantVisualMeshes(Transform transform)
        {
            return HasRobotBoundaryComponent(transform);
        }

        static bool BelongsToCurrentLink(Transform link, Transform candidate)
        {
            if (HasRobotBoundaryComponent(candidate))
                return false;

            var current = candidate.parent;
            while (current != null && current != link)
            {
                if (string.Equals(current.name, "Collisions", StringComparison.OrdinalIgnoreCase))
                    return false;

                if (HasRobotBoundaryComponent(current))
                    return false;

                current = current.parent;
            }

            return current == link;
        }

        static bool ContainsRobotBoundary(Transform transform)
        {
            if (HasRobotBoundaryComponent(transform))
                return true;

            for (var i = 0; i < transform.childCount; i++)
            {
                if (ContainsRobotBoundary(transform.GetChild(i)))
                    return true;
            }

            return false;
        }

        static bool HasRobotBoundaryComponent(Transform transform)
        {
            return transform.GetComponent<ArticulationBody>() != null
                || transform.GetComponent<UrdfRobot>() != null
                || transform.GetComponent<UrdfLink>() != null
                || transform.GetComponent<UrdfJoint>() != null
                || transform.GetComponent<UrdfInertial>() != null;
        }

        XmlElement Append(XmlElement parent, string name)
        {
            var child = Element(name);
            parent.AppendChild(child);
            return child;
        }

        XmlElement Element(string name)
        {
            return _doc.CreateElement(name);
        }

        string UniqueName(string name)
        {
            var safe = SafeName(string.IsNullOrEmpty(name) ? "unnamed" : name);
            var candidate = safe;
            var index = 1;
            while (_usedNames.Contains(candidate))
            {
                candidate = safe + "_" + index;
                index++;
            }

            _usedNames.Add(candidate);
            return candidate;
        }

        static string SafeName(string name)
        {
            var builder = new StringBuilder(name.Length);
            foreach (var c in name)
                builder.Append(char.IsLetterOrDigit(c) || c == '_' || c == '-' ? c : '_');

            return builder.ToString().Trim('_');
        }
    }

    static Vector3 Mj(Vector3 unity)
    {
        return new Vector3(unity.z, -unity.x, unity.y);
    }

    static Vector3 MjSize(Vector3 unitySize)
    {
        var size = Abs(unitySize);
        return new Vector3(size.z, size.x, size.y);
    }

    static Vector3 MjInertia(Vector3 unityInertia, Vector3 unityScale)
    {
        var inertia = Abs(unityInertia);
        var scale = Abs(unityScale);
        inertia = new Vector3(
            inertia.x * scale.x * scale.x,
            inertia.y * scale.y * scale.y,
            inertia.z * scale.z * scale.z);

        return FixInertiaTriangle(new Vector3(inertia.z, inertia.x, inertia.y));
    }

    static Vector3 FixInertiaTriangle(Vector3 inertia)
    {
        const float minInertia = 1e-9f;
        inertia = new Vector3(
            Mathf.Max(inertia.x, minInertia),
            Mathf.Max(inertia.y, minInertia),
            Mathf.Max(inertia.z, minInertia));

        var sumXY = inertia.x + inertia.y;
        var sumXZ = inertia.x + inertia.z;
        var sumYZ = inertia.y + inertia.z;
        if (sumXY < inertia.z)
            inertia.z = sumXY * 0.999f;
        if (sumXZ < inertia.y)
            inertia.y = sumXZ * 0.999f;
        if (sumYZ < inertia.x)
            inertia.x = sumYZ * 0.999f;

        return inertia;
    }

    static Quaternion Mj(Quaternion unity)
    {
        var rotation = Matrix4x4.Rotate(unity);

        var xAxis = Mj(rotation.MultiplyVector(Vector3.forward));
        var yAxis = Mj(rotation.MultiplyVector(-Vector3.right));
        var zAxis = Mj(rotation.MultiplyVector(Vector3.up));

        var converted = new Matrix4x4();
        converted.SetColumn(0, new Vector4(xAxis.x, xAxis.y, xAxis.z, 0f));
        converted.SetColumn(1, new Vector4(yAxis.x, yAxis.y, yAxis.z, 0f));
        converted.SetColumn(2, new Vector4(zAxis.x, zAxis.y, zAxis.z, 0f));
        converted[3, 3] = 1f;
        return QuaternionFromMatrix(converted);
    }

    static Quaternion QuaternionFromMatrix(Matrix4x4 matrix)
    {
        var forward = new Vector3(matrix.m02, matrix.m12, matrix.m22);
        var upwards = new Vector3(matrix.m01, matrix.m11, matrix.m21);
        return Quaternion.LookRotation(forward, upwards);
    }

    static Vector3 Abs(Vector3 vector)
    {
        return new Vector3(Mathf.Abs(vector.x), Mathf.Abs(vector.y), Mathf.Abs(vector.z));
    }

    static string Vec(Vector3 value)
    {
        return $"{Number(value.x)} {Number(value.y)} {Number(value.z)}";
    }

    static string Quat(Quaternion value)
    {
        return $"{Number(value.w)} {Number(value.x)} {Number(value.y)} {Number(value.z)}";
    }

    static string Number(float value)
    {
        return value.ToString("G9", Invariant);
    }
}
