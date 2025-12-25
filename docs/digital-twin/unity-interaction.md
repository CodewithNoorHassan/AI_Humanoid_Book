# Unity for Human-Robot Interaction

## Introduction to Unity for Robotics

Unity is a powerful game engine that provides high-fidelity visual rendering capabilities. In the context of digital twins, Unity excels at creating visually realistic environments and intuitive human-robot interaction interfaces.

## Setting up Unity for Robotics Applications

### Unity Installation and Configuration

Unity Hub is the recommended way to install and manage Unity versions. For robotics applications, we recommend using Unity 2022.3 LTS or newer for stability and long-term support.

1. **Download Unity Hub** from the official Unity website
2. **Install Unity Editor** version 2022.3 LTS or newer
3. **Install required modules**:
   - Android Build Support (if targeting mobile)
   - Windows Build Support (for Windows standalone builds)
   - Linux Build Support (for Linux standalone builds)
   - Visual Studio Tools for Unity

4. **Project Setup**:
   - Create a new 3D project
   - Set the project to use the Universal Render Pipeline (URP) or High Definition Render Pipeline (HDRP) for better visual quality
   - Configure the project settings for real-time performance

### Robotics-Specific Packages

Key Unity packages and tools for robotics applications:

#### Unity Robotics Hub
The Unity Robotics Hub provides tools for robotics simulation and development:
- **ROS# (ROS Sharp)**: Bridge between Unity and ROS/ROS2
- **Unity Robotics Package**: Tools for robotics simulation
- **ML-Agents**: For AI and machine learning in robotics

#### Installation via Package Manager:
1. Open Window → Package Manager
2. Click the + button → Add package from git URL
3. Add packages like:
   - `com.unity.robotics.ros-tcp-connector`
   - `com.unity.robotics.urdf-importer`
   - `com.unity.ml-agents`

### Required Dependencies
- **Python 3.x** (for ROS bridge)
- **ROS/ROS2** (for communication with simulation backends)
- **Git LFS** (for large asset support)

## Creating Realistic Visual Environments

### Material and Lighting Systems

Creating realistic materials with proper lighting to match real-world conditions. Unity's physically-based rendering (PBR) system allows for realistic material properties that respond appropriately to lighting.

#### Setting up PBR Materials

Unity uses a Metallic-Roughness workflow for PBR materials. Key properties include:

- **Albedo (Base Color)**: The base color of the material without lighting effects
- **Metallic**: How metallic the surface appears (0 = non-metallic, 1 = metallic)
- **Smoothness/Roughness**: How smooth or rough the surface is (affects reflections)
- **Normal Map**: Simulates surface details without adding geometry
- **Occlusion**: Simulates shadowing from ambient light
- **Emission**: Makes surfaces appear to emit light

#### Example Material Setup for Robot Components

```csharp
// Example C# script for configuring robot materials at runtime
using UnityEngine;

public class RobotMaterialSetup : MonoBehaviour
{
    [Header("Material Properties")]
    public Color robotColor = Color.gray;
    public float metallicValue = 0.8f;
    public float smoothnessValue = 0.6f;

    [Header("Texture References")]
    public Texture2D normalMap;
    public Texture2D occlusionMap;

    void Start()
    {
        // Get all renderers in the robot hierarchy
        Renderer[] renderers = GetComponentsInChildren<Renderer>();

        foreach (Renderer renderer in renderers)
        {
            // Create or modify material
            Material material = new Material(Shader.Find("Universal Render Pipeline/Lit"));

            // Set PBR properties
            material.SetColor("_BaseColor", robotColor);
            material.SetFloat("_Metallic", metallicValue);
            material.SetFloat("_Smoothness", smoothnessValue);

            // Assign textures if available
            if (normalMap != null)
                material.SetTexture("_BumpMap", normalMap);
            if (occlusionMap != null)
                material.SetTexture("_OcclusionMap", occlusionMap);

            renderer.material = material;
        }
    }
}
```

#### Lighting Setup for Robotics Environments

Realistic lighting is crucial for creating believable robot visualizations:

1. **Directional Light**: Simulates sunlight or primary light source
   - Intensity: 2-3 for outdoor scenes, 1-1.5 for indoor
   - Color temperature: 6500K for daylight, 3200K for warm light
   - Shadow settings: Enable shadows with appropriate resolution

2. **Environment Lighting**: Provides ambient illumination
   - Set to "From Skybox" with realistic skybox
   - Adjust intensity multiplier (0.1-0.5 typical)

3. **Additional Lights**: Fill lights or specific area lighting
   - Area lights for soft shadows
   - Spotlights for focused illumination

#### Example Lighting Configuration

```csharp
// Example C# script for setting up realistic lighting
using UnityEngine;
using UnityEngine.Rendering;

public class RealisticLightingSetup : MonoBehaviour
{
    [Header("Light Settings")]
    public float sunIntensity = 2.5f;
    public Color sunColor = new Color(1f, 0.95f, 0.8f, 1f);
    public float ambientIntensity = 0.2f;

    [Header("Shadow Settings")]
    public float shadowDistance = 50f;
    public int shadowResolution = 2048;

    void Start()
    {
        SetupDirectionalLight();
        SetupEnvironmentLighting();
        ConfigureShadowSettings();
    }

    void SetupDirectionalLight()
    {
        // Find or create directional light
        Light sunLight = FindObjectOfType<Light>();
        if (sunLight == null || sunLight.type != LightType.Directional)
        {
            GameObject lightObj = new GameObject("Sun Light");
            sunLight = lightObj.AddComponent<Light>();
            sunLight.type = LightType.Directional;
        }

        sunLight.color = sunColor;
        sunLight.intensity = sunIntensity;
        sunLight.transform.rotation = Quaternion.Euler(50f, -30f, 0f);

        // Configure shadows
        sunLight.shadows = LightShadows.Soft;
        sunLight.shadowStrength = 0.8f;
    }

    void SetupEnvironmentLighting()
    {
        RenderSettings.ambientIntensity = ambientIntensity;
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Trilight;
    }

    void ConfigureShadowSettings()
    {
        QualitySettings.shadowDistance = shadowDistance;
        QualitySettings.shadowResolution = (ShadowResolution) Mathf.Clamp(shadowResolution / 512, 0, 3);
    }
}
```

### Environmental Design

Building environments that accurately represent the robot's real-world operating conditions. Consider the following elements:

- **Scale**: Ensure environment objects are correctly scaled relative to the robot
- **Physics**: Set up colliders for accurate interaction visualization
- **Details**: Add small details that make the environment feel real (dust, wear patterns)
- **Context**: Include elements that provide context for the robot's purpose

### Performance Optimization

Balancing visual quality with real-time performance requirements:

- **Level of Detail (LOD)**: Use LOD groups to reduce geometry complexity at distance
- **Occlusion Culling**: Hide objects not visible to the camera
- **Light Baking**: Bake static lighting to reduce real-time calculations
- **Texture Compression**: Use appropriate texture formats for performance
- **Object Pooling**: Reuse objects instead of creating/destroying frequently

## Human-Robot Interaction Interfaces

### Visualization Techniques

Different approaches to visualizing robot state, sensor data, and planned behaviors.

#### Robot State Visualization

Visualizing the current state of the robot using various techniques:

1. **Joint Position Indicators**: Show the current position of each joint with color coding
2. **Sensor Data Overlay**: Display sensor readings directly on the robot model
3. **Path Visualization**: Show planned paths and trajectories with lines or markers
4. **Status Indicators**: Use lights, colors, or animations to show operational status

#### Example: Joint Position Visualization Script

```csharp
// Example C# script for visualizing joint positions
using UnityEngine;

public class JointPositionVisualizer : MonoBehaviour
{
    [Header("Joint Configuration")]
    public Transform[] joints;  // Array of joint transforms
    public GameObject indicatorPrefab;  // Prefab for joint indicators
    public Color activeColor = Color.green;
    public Color inactiveColor = Color.red;

    [Header("Threshold Settings")]
    public float velocityThreshold = 0.1f;  // Movement threshold

    private GameObject[] indicators;
    private Vector3[] previousPositions;

    void Start()
    {
        InitializeIndicators();
    }

    void Update()
    {
        UpdateJointIndicators();
    }

    void InitializeIndicators()
    {
        indicators = new GameObject[joints.Length];
        previousPositions = new Vector3[joints.Length];

        for (int i = 0; i < joints.Length; i++)
        {
            // Store initial positions
            previousPositions[i] = joints[i].position;

            // Create indicator object
            indicators[i] = Instantiate(indicatorPrefab, joints[i]);
            indicators[i].transform.localPosition = Vector3.zero;
        }
    }

    void UpdateJointIndicators()
    {
        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i] != null)
            {
                // Calculate velocity
                float velocity = Vector3.Distance(joints[i].position, previousPositions[i]) / Time.deltaTime;
                previousPositions[i] = joints[i].position;

                // Update indicator color based on movement
                Renderer indicatorRenderer = indicators[i].GetComponent<Renderer>();
                if (indicatorRenderer != null)
                {
                    indicatorRenderer.material.color = velocity > velocityThreshold ? activeColor : inactiveColor;
                }
            }
        }
    }
}
```

#### Sensor Data Visualization

Visualizing sensor data in real-time:

- **LiDAR Point Clouds**: Display point clouds in the 3D view
- **Camera Feeds**: Show camera images as textures on UI panels
- **IMU Data**: Visualize orientation with rotating coordinate frames
- **Force/Torque**: Show force vectors as arrows on the robot

### Control Interfaces

Creating intuitive interfaces for human operators to interact with the digital twin.

#### Example Control Interface

```csharp
// Example C# script for robot control interface
using UnityEngine;
using UnityEngine.UI;

public class RobotControlInterface : MonoBehaviour
{
    [Header("Robot Reference")]
    public GameObject robot;  // The robot to control

    [Header("Control Elements")]
    public Slider linearVelocitySlider;
    public Slider angularVelocitySlider;
    public Button moveForwardButton;
    public Button moveBackwardButton;
    public Button rotateLeftButton;
    public Button rotateRightButton;

    [Header("Movement Settings")]
    public float maxLinearSpeed = 2.0f;
    public float maxAngularSpeed = 1.0f;

    private Rigidbody robotRigidbody;

    void Start()
    {
        robotRigidbody = robot.GetComponent<Rigidbody>();

        // Setup button listeners
        SetupButtonListeners();
    }

    void SetupButtonListeners()
    {
        if (moveForwardButton != null)
            moveForwardButton.onClick.AddListener(() => MoveRobot(Vector3.forward));
        if (moveBackwardButton != null)
            moveBackwardButton.onClick.AddListener(() => MoveRobot(Vector3.back));
        if (rotateLeftButton != null)
            rotateLeftButton.onClick.AddListener(() => RotateRobot(-1));
        if (rotateRightButton != null)
            rotateRightButton.onClick.AddListener(() => RotateRobot(1));
    }

    void MoveRobot(Vector3 direction)
    {
        if (robotRigidbody != null)
        {
            float speed = linearVelocitySlider.value * maxLinearSpeed;
            robotRigidbody.AddForce(robot.transform.TransformDirection(direction) * speed);
        }
    }

    void RotateRobot(int direction)
    {
        if (robotRigidbody != null)
        {
            float torque = angularVelocitySlider.value * maxAngularSpeed * direction;
            robotRigidbody.AddTorque(Vector3.up * torque);
        }
    }
}
```

### Data Presentation

Effectively presenting complex robotics data in a comprehensible visual format:

- **Dashboards**: Create real-time dashboards showing key metrics
- **Graphs and Charts**: Visualize sensor data over time
- **Color Coding**: Use colors to represent different data values
- **3D Overlays**: Show data directly in the 3D view (paths, sensor ranges, etc.)

## Integration with Simulation Backends

### Connecting Unity to Gazebo

Methods for synchronizing Unity visualization with Gazebo physics simulation.

### Real-time Data Streaming

Ensuring low-latency updates between the physics simulation and visual representation.

### Coordinate System Alignment

Maintaining consistent coordinate systems across different simulation components.

## Advanced Interaction Scenarios

### Multi-Robot Visualization

Handling visualization of multiple robots in the same environment.

### Scenario Playback

Creating tools for replaying recorded robot behaviors and interactions.

### User Studies and Data Collection

Using Unity environments for conducting human-robot interaction studies.

## Best Practices

### Visual Fidelity vs. Performance

Balancing visual quality with computational efficiency. For digital twin applications, finding the right balance is crucial for maintaining real-time performance while providing adequate visual information.

#### Performance Optimization Strategies

1. **Level of Detail (LOD) Systems**: Implement LOD groups to automatically switch between high and low-detail models based on distance:

```csharp
// Example LOD setup script
using UnityEngine;

public class RobotLODSetup : MonoBehaviour
{
    [Header("LOD Configuration")]
    public float[] lodDistances = { 10f, 30f, 60f };  // Distance thresholds
    public Renderer[] highDetailRenderers;
    public Renderer[] mediumDetailRenderers;
    public Renderer[] lowDetailRenderers;

    private LODGroup lodGroup;

    void Start()
    {
        SetupLOD();
    }

    void SetupLOD()
    {
        lodGroup = gameObject.AddComponent<LODGroup>();

        LOD[] lods = new LOD[3];

        // LOD 0: High detail (closest)
        lods[0] = new LOD(0.5f, highDetailRenderers);
        lods[0].screenRelativeTransitionHeight = lodDistances[0] / 1000f;

        // LOD 1: Medium detail
        lods[1] = new LOD(0.25f, mediumDetailRenderers);
        lods[1].screenRelativeTransitionHeight = lodDistances[1] / 1000f;

        // LOD 2: Low detail (farthest)
        lods[2] = new LOD(0.01f, lowDetailRenderers);
        lods[2].screenRelativeTransitionHeight = lodDistances[2] / 1000f;

        lodGroup.SetLODs(lods);
        lodGroup.RecalculateBounds();
    }
}
```

2. **Occlusion Culling**: Hide objects not visible to the camera to save rendering resources.

3. **Texture Streaming**: Load textures at appropriate resolutions based on viewing distance.

4. **Shader Optimization**: Use simpler shaders when full visual fidelity isn't required.

#### Quality Settings for Robotics Applications

Configure Unity's quality settings for optimal robotics visualization:

```csharp
// Example quality settings configuration
using UnityEngine;

public class RoboticsQualitySettings : MonoBehaviour
{
    void Start()
    {
        // Configure quality settings for robotics applications
        QualitySettings.vSyncCount = 0;  // Disable VSync for consistent frame timing
        Application.targetFrameRate = 60;  // Target 60 FPS for smooth visualization

        // Set appropriate shadow resolution based on performance needs
        QualitySettings.shadowResolution = ShadowResolution.Medium;
        QualitySettings.shadowDistance = 50f;  // Limit shadow distance for performance

        // Adjust anisotropic filtering for better texture quality at angles
        QualitySettings.anisotropicFiltering = AnisotropicFiltering.Enable;
    }
}
```

### User Experience Design

Creating interfaces that are intuitive for robotics researchers and operators. Consider the following principles:

- **Consistency**: Use consistent visual elements and interaction patterns
- **Feedback**: Provide immediate visual feedback for all user actions
- **Accessibility**: Ensure interfaces work well for users with different abilities
- **Efficiency**: Minimize the number of steps required to perform common tasks
- **Clarity**: Present information in a clear, unambiguous manner

### Validation Techniques

Ensuring that visual representations accurately reflect the underlying simulation state:

1. **Cross-verification**: Compare visual outputs with numerical data
2. **Reference comparison**: Compare with known benchmarks or real-world data
3. **Consistency checks**: Ensure visual elements update consistently with simulation time
4. **Coordinate system verification**: Verify that visual elements are positioned correctly in 3D space

## Exercises

### Exercise 1: Unity Environment Setup
Set up a Unity project for robotics applications:
1. Install Unity Hub and Unity Editor 2022.3 LTS
2. Create a new 3D project
3. Install the ROS TCP Connector package
4. Import the URDF Importer package
5. Verify that all required packages are properly installed

### Exercise 2: Basic Robot Visualization
Import and visualize a simple robot model:
1. Obtain a URDF file for a simple robot (e.g., a mobile base)
2. Import the URDF into Unity using the URDF Importer
3. Set up basic lighting and materials
4. Test the visualization with different lighting conditions

### Exercise 3: Material and Lighting Implementation
Create realistic materials and lighting for a robot:
1. Create PBR materials for different robot components (metal, plastic, rubber)
2. Implement proper metallic and smoothness values for each material
3. Set up realistic lighting for an indoor environment
4. Compare the visual quality with and without proper PBR materials
5. Document the performance impact of different material settings

### Exercise 4: Interactive Control Interface
Build an interactive control interface for the robot:
1. Create UI elements for controlling robot movement (sliders, buttons)
2. Implement scripts to control the robot based on user input
3. Add visual feedback for robot status (moving, stopped, error states)
4. Test the interface with different control scenarios
5. Evaluate the usability of your interface with others

### Exercise 5: Performance Optimization Challenge
Optimize a Unity scene for better performance:
1. Create a scene with multiple robot models and complex environment
2. Measure the frame rate and identify performance bottlenecks
3. Implement LOD systems for the robot models
4. Apply texture streaming and occlusion culling
5. Compare performance before and after optimization
6. Document the trade-offs between visual quality and performance

## Summary

Unity provides powerful capabilities for creating high-fidelity visual representations in digital twin systems. By properly setting up the Unity environment with robotics-specific packages and tools, we can create compelling visual interfaces that enhance human-robot interaction and provide intuitive visualization of complex robotics data.

This chapter has covered the complete workflow for implementing Unity-based human-robot interaction in digital twins, from initial environment setup to advanced performance optimization techniques. You've learned how to configure realistic materials and lighting using PBR systems, implement various visualization techniques for robot state and sensor data, create intuitive control interfaces, and optimize performance for real-time applications. The exercises have provided hands-on experience with environment setup, material implementation, interface creation, and performance optimization challenges.

With this chapter, we've completed the three-pillar approach to digital twin development: physics simulation with Gazebo, virtual sensor modeling, and Unity-based human-robot interaction. These components work together to create comprehensive digital twin systems that can accelerate robotics development and provide safe, cost-effective testing environments.

## Cross-References

- **Previous Chapter**: [Virtual Sensor Modeling](virtual-sensors.md) - Sensor data visualization and integration with visual systems
- **Previous Chapter**: [Physics Simulation with Gazebo](gazebo-physics.md) - Physics simulation visualization and integration
- **Previous Chapter**: [Introduction to Digital Twins](intro.md) - Overview of the complete digital twin architecture

## Hands-on Exercise

Create a Unity scene with a humanoid robot in a realistic environment, implementing interactive controls and real-time visualization of sensor data.