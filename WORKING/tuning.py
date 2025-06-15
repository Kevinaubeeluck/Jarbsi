import matplotlib.pyplot as plt

# Example data: interleaved input/output lines
data = """
input:0.00
output:0.000000
input:0.00
output:0.132599
input:0.00
output:0.666893
input:0.00
output:1.111488
input:0.00
output:1.533982
input:0.00
output:1.966876
input:0.00
output:2.434868
input:0.00
output:3.550247
input:0.00
output:4.055941
input:0.00
output:4.504428
input:0.00
output:-4.851804
input:0.00
output:-19.634954
input:0.00
output:-19.634954
input:0.00
output:-19.634954
input:0.00
output:-19.634954
input:0.00
output:-19.634954
input:10.00
output:-19.634954
input:10.00
output:-19.634954
input:10.00
output:-19.634954
input:0.00
output:-19.634954
input:0.00
output:-19.634954
input:-10.00
output:-19.634954
input:-10.00
output:-19.634954
input:0.00
output:-19.634954
input:0.00
output:-17.401176
input:0.00
output:-4.322276
input:0.00
output:-14.764879
input:0.00
output:-14.981205
input:0.00
output:-18.414669
input:0.00
output:-15.840082
input:-10.00
output:-19.634954
input:-10.00
output:-19.634954
input:0.00
output:-19.080364
input:10.00
output:-5.303288
input:10.00
output:18.773808
input:0.00
output:17.130093
input:10.00
output:18.697773
input:10.00
output:19.608055
input:-10.00
output:-5.415329
input:-10.00
output:-18.955395
input:0.00
output:-18.950083
input:10.00
output:2.101418
input:10.00
output:19.569798
input:0.00
output:19.574167
input:0.00
output:19.420778
input:-10.00
output:0.663416
input:-10.00
output:-19.445780
input:0.00
output:-19.235514
input:0.00
output:-18.961521
input:0.00
output:-19.536470
input:0.00
output:-15.860261
"""

# Parse the data
inputs = []
outputs = []

for line in data.strip().splitlines():
    if line.startswith("input:"):
        inputs.append(float(line.split(":")[1]))
    elif line.startswith("output:"):
        outputs.append(float(line.split(":")[1]))

# Generate x-axis values
x = range(len(inputs))  # assuming each input-output pair is sequential

# Plotting
plt.figure(figsize=(10, 5))
plt.plot(x, inputs, 'b--', label="Input (dotted red)")     # Dotted red line
plt.plot(x, outputs, 'r-', label="Output (solid red)")     # Solid red line
plt.xlabel("Time / Index")
plt.ylabel("Value")
plt.title("Input vs Output")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
