# ORB-SLAM3 Dataset Collection for Triangulation vs RGB-D Analysis

## Overview

This implementation adds dataset collection capabilities to ORB-SLAM3 to capture triangulated vs RGB-D depth comparisons at every Local Bundle Adjustment iteration. This is specifically designed for research analyzing the accuracy of geometric triangulation against direct RGB-D measurements.

## Key Features

- **Local BA Only**: Collects data exclusively from Local Bundle Adjustment (not Full BA or Loop Closing)
- **Frame-wise Structure**: Organizes data by frame rather than by point for easier analysis
- **Thread-safe**: Uses proper mutex handling for concurrent data collection
- **Triangulation Analysis**: Compares geometric triangulation with RGB-D direct depth measurements
- **Observability Verification**: Checks if optimized points remain observable after BA
- **Validity Filtering**: Applies comprehensive data quality checks

## Architecture

### Core Components

1. **DatasetCollector** (`include/DatasetCollector.h`, `src/DatasetCollector.cc`)
   - Singleton pattern for global access
   - Thread-safe data collection
   - JSON output format with metadata

2. **LocalMapping Integration** (`src/LocalMapping.cc`)
   - Pre-BA data collection (lines 171-172)
   - Post-BA data collection (lines 205-207)
   - Automatic initialization for RGB-D modes

### Data Structure

```json
{
  "frame_id": int,
  "timestamp": float,
  "frame_pose": [[4x4 matrix]],
  "is_keyframe": bool,
  "ba_iteration": int,
  "points": [
    {
      "point_id": int,
      "triangulated_xyz": [x, y, z],
      "rgbd_xyz": [x, y, z],
      "triangulation_angle_degrees": float,
      "optimized_xyz": [x, y, z],
      "num_observations": int,
      "is_observable": bool,
      "is_valid": bool
    }
  ]
}
```

## Installation Requirements

### System Dependencies

```bash
# Install JsonCpp library
sudo apt-get install libjsoncpp-dev pkg-config

# Standard ORB-SLAM3 dependencies
sudo apt-get install libeigen3-dev libopencv-dev libpangolin-dev
```

### Build Instructions

1. **Clone and prepare ORB-SLAM3**:
   ```bash
   cd ORB_SLAM3
   chmod +x build.sh
   ```

2. **Build with dataset collection support**:
   ```bash
   ./build.sh
   ```

3. **Verify JsonCpp linking**:
   ```bash
   ldd lib/libORB_SLAM3.so | grep jsoncpp
   ```

## Usage

### Automatic Activation

Dataset collection is automatically enabled for RGB-D modes:
- RGB-D (`System::RGBD`)
- RGB-D Inertial (`System::IMU_RGBD`)

### Output Location

Data is saved to `./dataset_output/` relative to the executable directory:

```
./dataset_output/
├── metadata.json
├── frame_000001_localba0001.json
├── frame_000001_localba0001_postBA.json
├── frame_000002_localba0002.json
├── frame_000002_localba0002_postBA.json
└── ...
```

### File Naming Convention

- **Pre-BA**: `frame_{frame_id}_localba{ba_iteration}.json`
- **Post-BA**: `frame_{frame_id}_localba{ba_iteration}_postBA.json`
- **Metadata**: `metadata.json`

## Data Collection Process

### 1. Pre-BA Collection (LocalMapping.cc:171-172)

- Identifies local map points for current Local BA
- Attempts triangulation between current and last keyframe
- Records initial triangulated positions
- Captures RGB-D ground truth measurements
- Stores pre-optimization point positions

### 2. Local Bundle Adjustment Execution

- Standard ORB-SLAM3 Local BA runs normally
- Only Local BA is monitored (Full BA ignored)
- Optimization updates point positions

### 3. Post-BA Collection (LocalMapping.cc:205-207)

- Records optimized point positions after Local BA
- Verifies observability of optimized points
- Applies data validity checks
- Saves frame data with post-optimization results

## Configuration Parameters

### Default Thresholds

```cpp
// Camera depth range
MIN_DEPTH_DEFAULT = 0.3f        // 30cm minimum
MAX_DEPTH_DEFAULT = 10.0f       // 10m maximum

// Triangulation quality
MIN_TRIANGULATION_ANGLE = 1.0f  // 1 degree minimum angle

// Error threshold for analysis
ERROR_THRESHOLD = 0.1f          // 10cm error threshold
```

### Customization

Modify thresholds programmatically:

```cpp
DatasetCollector::GetInstance().SetDepthThresholds(0.5f, 8.0f);
DatasetCollector::GetInstance().SetTriangulationThreshold(2.0f);
```

## Validation Checks

### Triangulation Validity
- Minimum triangulation angle (default: 1°)
- No parallel ray conditions
- Sufficient baseline between frames
- Point in front of both cameras

### RGB-D Validity
- Depth not NaN or infinity
- Depth within sensor range
- No reflective/transparent surface artifacts

### Point Quality
- Minimum 2 observations
- Not marked as outlier in ORB-SLAM3
- Passes local map point tests

## Output Analysis

### Metadata File
```json
{
  "description": "ORB-SLAM3 Local Bundle Adjustment Dataset",
  "collection_type": "Local BA only",
  "camera": { "fx": 525.0, "fy": 525.0, "cx": 319.5, "cy": 239.5 },
  "thresholds": { "min_depth": 0.3, "max_depth": 10.0 },
  "total_ba_iterations": 156
}
```

### Analysis Scripts
Expected use cases:
1. **Triangulation vs RGB-D error analysis**
2. **Bundle adjustment convergence studies**
3. **Camera calibration validation**
4. **Depth sensor accuracy assessment**

## Performance Impact

- **Minimal overhead**: Data collection only during Local BA
- **No tracking interference**: Collection runs in Local Mapping thread
- **Selective activation**: Only enabled for RGB-D modes
- **Efficient storage**: JSON streaming prevents memory buildup

## Troubleshooting

### Common Issues

1. **JsonCpp not found**:
   ```bash
   sudo apt-get install libjsoncpp-dev
   ```

2. **Permission denied**:
   ```bash
   mkdir -p ./dataset_output
   chmod 755 ./dataset_output
   ```

3. **No data collected**:
   - Verify RGB-D mode is active
   - Check console for DatasetCollector initialization messages
   - Ensure Local BA is running (requires >2 keyframes)

### Debug Output
```
[LocalMapping] DatasetCollector initialized for RGB-D mode
[DatasetCollector] Pre-BA: Frame 1 BA iteration 1 Points: 45
[DatasetCollector] Post-BA: Frame 1 BA iteration 1 Points: 43
[DatasetCollector] Finalized - Total BA iterations: 156
```

## Research Applications

This dataset enables research in:

1. **Geometric vs Direct Depth Comparison**
   - Accuracy analysis across different scene types
   - Error characterization by distance/angle

2. **Bundle Adjustment Analysis**
   - Convergence behavior studies
   - Point position optimization tracking

3. **Sensor Fusion Research**
   - RGB-D + triangulation combinations
   - Confidence weighting strategies

4. **SLAM Algorithm Development**
   - Improved depth estimation methods
   - Enhanced uncertainty modeling

## Implementation Notes

- **Thread Safety**: All DatasetCollector operations are mutex-protected
- **Memory Efficient**: Direct JSON streaming prevents large memory usage
- **Backward Compatible**: No changes to existing ORB-SLAM3 functionality
- **Research Focus**: Designed specifically for Local BA analysis
- **Extensible**: Easy to add new data fields or collection points

## Version Information

- **ORB-SLAM3 Base**: Compatible with standard ORB-SLAM3 release
- **Dataset Collection**: Added September 2025
- **Target Use**: Research and algorithm development
- **License**: Follows ORB-SLAM3 license terms