from fabrikSolver import FabrikSolver2D, FabrikSolver3D

arm = FabrikSolver3D()

arm.addSegment(100, 200, 100.0)
arm.addSegment(100, -20, 200)
arm.addSegment(100, -20, 0)
arm.addSegment(100, 20, 0)

# arm.plot()

for segment in arm.segments:
    print(segment.point)
arm.compute(100, 150, 50)
for segment in arm.segments:
    print(segment.point)

arm.plot()

# arm = FabrikSolver2D()

# arm.addSegment(100, 30)
# arm.addSegment(100, 70)

# arm.compute(100, 70)

# arm.plot()