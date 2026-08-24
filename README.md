

# NURBS2D Isoline Intersection Solver

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://isocpp.org/std/the-standard)

A high-performance, header-only C++20 library for computing intersections between NURBS curves and iso-parametric lines (axis-aligned lines in 2D space).

## Features

- **Header-only**: Just include and use, no compilation needed
- **High Performance**: Hybrid Newton-Raphson + bisection root finding
- **Robust Algorithm**: Handles edge cases, tangent intersections, and multiple roots
- **Flexible**: Supports both rational and non-rational NURBS curves

## Requirements

- C++20 compatible compiler (GCC 10+, Clang 10+, MSVC 2019+)
- [tinynurbs](https://github.com/pradeep-pyro/tinynurbs) - NURBS curve/surface library
- [glm](https://github.com/g-truc/glm) - OpenGL Mathematics library

### Optional (for tests and visualization)

- [Catch2](https://github.com/catchorg/Catch2) v3 - Testing framework
- [sciplot](https://github.com/sciplot/sciplot) - Plotting library (requires gnuplot)

## Installation

This is a header-only library. Simply copy `include/nurbs2d_iso_intersect.h` to your project and include it:

```cpp
#include "nurbs2d_iso_intersect.h"
```

### Using xmake

```lua
add_requires("glm ~0.9.9")

target("your_project")
    set_kind("binary")
    set_languages("c++20")
    add_packages("glm")
    add_includedirs("path/to/tinynurbs/include")
    add_includedirs("path/to/nurbs2d-iso-intersect/include")
```

### Using CMake

```cmake
find_package(glm REQUIRED)
target_include_directories(your_target PRIVATE 
    path/to/tinynurbs/include
    path/to/nurbs2d-iso-intersect/include
)
target_compile_features(your_target PRIVATE cxx_std_20)
```

## Quick Start

```cpp
#include <nurbs2d_iso_intersect.h>

using namespace nurbs2d;

// Create a quadratic Bezier curve
tinynurbs::Curve<double> curve;
curve.degree = 2;
curve.knots = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
curve.control_points = {
    glm::dvec3(0.0, 0.0, 0.0),
    glm::dvec3(0.5, 1.0, 0.0),
    glm::dvec3(1.0, 0.0, 0.0)
};

// Find intersections with x = 0.5 and y = 0.25
std::vector<double> vertical_lines = {0.5};
std::vector<double> horizontal_lines = {0.25};
std::vector<double> result_params;

IntersectCurveWithIsolines(curve, nullptr, &vertical_lines, &horizontal_lines, result_params);

// Evaluate intersection points
for (double t : result_params) {
    Vec2 pt = EvaluateCurvePoint(curve, t);
    std::cout << "t=" << t << " -> (" << pt.x << ", " << pt.y << ")\n";
}
```

## API Reference

### Core Functions

#### `IntersectCurveWithIsolines`

Find all intersections between a curve and multiple iso-lines:

```cpp
template <typename CurveType>
void IntersectCurveWithIsolines(
    const CurveType& curve,                          // The curve to intersect
    const ParameterInterval* search_domain,          // Search domain (nullptr = full curve domain)
    const std::vector<double>* vertical_lines,       // Vertical lines x = const (nullable)
    const std::vector<double>* horizontal_lines,     // Horizontal lines y = const (nullable)
    std::vector<double>& result_params,              // Output: intersection parameters (reference)
    double tolerance = kDefaultTolerance);           // Root finding tolerance (default: 1e-6)
```

#### `IntersectCurveWithIsoline`

Find intersections with a single iso-line:

```cpp
template <typename CurveType>
void IntersectCurveWithIsoline(
    const CurveType& curve,
    double iso_value,
    IsoLineType iso_type,                            // kVertical or kHorizontal
    std::vector<double>& result_params,              // Output: intersection parameters (reference)
    double tolerance = kDefaultTolerance);
```

### Types

```cpp
namespace nurbs2d {

// 2D vector type
using Vec2 = glm::dvec2;

// NURBS curve alias
template <typename T>
using Curve2D = tinynurbs::Curve<T>;

// Parameter interval [min, max]
struct ParameterInterval {
    double start;
    double end;
    
    [[nodiscard]] double Length() const;
    [[nodiscard]] double Midpoint() const;
    [[nodiscard]] bool Contains(double t) const;
    [[nodiscard]] double Clamp(double t) const;
};

// Iso-line direction
enum class IsoLineType { kVertical, kHorizontal };

// Constants
inline constexpr double kDefaultTolerance = 1e-6;
inline constexpr int kMaxNewtonIterations = 50;
inline constexpr int kMaxRecursionDepth = 64;

}  // namespace nurbs2d
```

### Solver Class

For fine-grained control over the root-finding process:

```cpp
template <typename CurveType>
class IsoIntersectionSolver {
public:
    IsoIntersectionSolver(
        const CurveType& curve,
        ParameterInterval domain,
        IsoLineType iso_type,
        double iso_value,
        double tolerance = kDefaultTolerance);

    // Root-finding methods return std::optional<double>
    [[nodiscard]] std::optional<double> FindRootNewton(double initial_guess) const;
    [[nodiscard]] std::optional<double> FindRootBisection() const;
    [[nodiscard]] std::optional<double> FindRootHybrid() const;

    // Collect all intersections in the domain
    void CollectIntersections(std::vector<double>& result_params) const;
};
```

## Advanced Usage

### Custom Tolerance

```cpp
// Use tighter tolerance for precision-critical applications
IntersectCurveWithIsolines(curve, nullptr, &vertical_lines, &horizontal_lines, result_params, 1e-10);
```

### Search in Specific Domain

```cpp
// Only search in parameter range [0.2, 0.8]
ParameterInterval domain{0.2, 0.8};
IntersectCurveWithIsolines(curve, &domain, &vertical_lines, &horizontal_lines, result_params);
```

### Direct Solver Access

```cpp
// For fine-grained control over root finding
IsoIntersectionSolver<tinynurbs::Curve<double>> solver(
    curve, domain, IsoLineType::kVertical, 0.5);

if (auto root = solver.FindRootHybrid()) {
    std::cout << "Found intersection at parameter " << *root << "\n";
}
```

## Building Tests

```bash
# Using xmake
xmake
xmake run tests        # Unit tests (77 assertions, 15 test cases)
xmake run benchmark    # Performance benchmarks
xmake run test_visual  # Visualization (requires gnuplot)
```

## Project Structure

```
├── include/
│   └── nurbs2d_iso_intersect.h   # Main header-only library (~800 lines)
├── tests/
│   ├── test_iso_intersection.cpp        # Unit tests
│   ├── benchmark_iso_intersection.cpp   # Benchmarks
│   └── test_iso_intersection_visual.cpp # Visualization
├── third_party/
│   ├── tinynurbs/                # NURBS library
│   └── sciplot/                  # Plotting library
├── xmake.lua                     # Build configuration
└── README.md
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- [tinynurbs](https://github.com/pradeep-pyro/tinynurbs) by Pradeep Kumar Jayaraman
- [glm](https://github.com/g-truc/glm) by G-Truc Creation
- [sciplot](https://github.com/sciplot/sciplot) for visualization support
