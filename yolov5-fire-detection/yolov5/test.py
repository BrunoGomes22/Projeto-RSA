coords = (10, 20, 30, 40)
mission = f"""Mission coordinates:
// Define a set of coordinates
coords = [

"""

for x in range(coords[0], coords[2] + 1, 10):
    for y in range(coords[1], coords[3] + 1, 10):
        mission += f"    ({x}, {y}),\n"
mission += """
]


with open("output.txt", "w") as f:
    f.write(mission)