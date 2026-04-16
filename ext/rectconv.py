p1_x, p1_y, p1_z = map(lambda p: float(p), input("First point: ").split(" "))
p2_x, p2_y, p2_z = map(lambda p: float(p), input("Second point: ").split(" "))

print(f"Position: {(p1_x + p2_x) / 2.0} {(p1_y + p2_y) / 2.0} {(p1_z + p2_z) / 2.0}")
print(
    f"Size: {abs(p2_x - p1_x) / 2.0} {abs(p2_y - p1_y) / 2.0} {abs(p2_z - p1_z) / 2.0}"
)
