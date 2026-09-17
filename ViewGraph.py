import matplotlib.pyplot as plt
import pandas as pd

# 1. Load the data, skipping the first PuTTY header line
# 'comment="="' automatically ignores lines starting with the header format
df = pd.read_csv(
    "putty.log",
    skiprows=1,
    header=None,
    comment="=",
    names=["Timestamp", "Status", "Val1", "Temp1", "Val2", "Temp2", "Val3", "Temp3"],
)
df["Timestamp"] = df["Timestamp"]/1000.0
# 2. Plot the data
plt.figure(figsize=(10, 6))

# Example: Plotting Val1, Val2, and Val3 over time
plt.plot(df["Timestamp"], df["Val1"], label="Sensor 1", marker="o")
plt.plot(df["Timestamp"], df["Val2"], label="Sensor 2", marker="s")
plt.plot(df["Timestamp"], df["Val3"], label="Sensor 3", marker="^")

# 3. Formatting
plt.title("PuTTY Log Data Visualization")
plt.xlabel("Timestamp (s)")
plt.ylabel("Values")
plt.grid(True, linestyle="--", alpha=0.6)
plt.legend()

# Display the plot
plt.tight_layout()
plt.show()
