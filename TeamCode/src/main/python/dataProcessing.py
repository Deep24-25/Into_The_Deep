import matplotlib.pyplot as plt
import pandas as pd

# Load your data
df = pd.read_csv("../python/data/data.csv")

# Separate the DataFrame based on color
dfYellow = df[df["Color"] == "Yellow"]
dfRed = df[df["Color"] == "Red"]
dfBlue = df[df["Color"] == "Blue"]


# ------------------------YELLOW LEFT---------------------------#

# Initialize the plot
plt.figure(figsize=(10, 6))

# Plot different columns for Yellow category
plt.plot(dfYellow.index, dfYellow["Red_left"], label="Red_left", color="red")
plt.plot(dfYellow.index, dfYellow["Green_left"], label="Green_left", color="green")
plt.plot(dfYellow.index, dfYellow["Blue_left"], label="Blue_left", color="blue")
plt.plot(dfYellow.index, dfYellow["Alpha_left"], label="Alpha_left", color="orange")

# Add labels, title, and legend
plt.xlabel("Index")
plt.ylabel("Values")
plt.title("Yellow Left")
plt.legend(loc="upper left")  # Legend inside the plot
plt.ylim(0, 500)

# Display the plot
plt.show()

# ------------------------YELLOW RIGHT---------------------------#

# Initialize the plot
plt.figure(figsize=(10, 6))

# Plot different columns for Yellow category
plt.plot(dfYellow.index, dfYellow["Red_right"], label="Red_Right", color="red")
plt.plot(dfYellow.index, dfYellow["Green_right"], label="Green_Right", color="green")
plt.plot(dfYellow.index, dfYellow["Blue_right"], label="Blue_Right", color="blue")
plt.plot(dfYellow.index, dfYellow["Alpha_right"], label="Alpha_Right", color="orange")

# Add labels, title, and legend
plt.xlabel("Index")
plt.ylabel("Values")
plt.title("Yellow Right")
plt.legend(loc="upper left")  # Legend inside the plot
plt.ylim(0, 500)
# Display the plot
plt.show()

# ------------------------RED LEFT---------------------------#

# Initialize the plot
plt.figure(figsize=(10, 6))

# Plot different columns for Yellow category
plt.plot(dfRed.index, dfRed["Red_left"], label="Red_left", color="red")
plt.plot(dfRed.index, dfRed["Green_left"], label="Green_left", color="green")
plt.plot(dfRed.index, dfRed["Blue_left"], label="Blue_left", color="blue")
plt.plot(dfRed.index, dfRed["Alpha_left"], label="Alpha_left", color="orange")

# Add labels, title, and legend
plt.xlabel("Index")
plt.ylabel("Values")
plt.title("Red Left")
plt.legend(loc="upper left")  # Legend inside the plot
plt.ylim(0, 500)

# Display the plot
plt.show()

# ------------------------RED RIGHT---------------------------#

# Initialize the plot
plt.figure(figsize=(10, 6))

# Plot different columns for Yellow category
plt.plot(dfRed.index, dfRed["Red_right"], label="Red_Right", color="red")
plt.plot(dfRed.index, dfRed["Green_right"], label="Green_Right", color="green")
plt.plot(dfRed.index, dfRed["Blue_right"], label="Blue_Right", color="blue")
plt.plot(dfRed.index, dfRed["Alpha_right"], label="Alpha_Right", color="orange")

# Add labels, title, and legend
plt.xlabel("Index")
plt.ylabel("Values")
plt.title("Red Right")
plt.legend(loc="upper left")  # Legend inside the plot
plt.ylim(0, 500)
# Display the plot
plt.show()

# ------------------------BLUE LEFT---------------------------#

# Initialize the plot
plt.figure(figsize=(10, 6))

# Plot different columns for Yellow category
plt.plot(dfBlue.index, dfBlue["Red_left"], label="Red_left", color="red")
plt.plot(dfBlue.index, dfBlue["Green_left"], label="Green_left", color="green")
plt.plot(dfBlue.index, dfBlue["Blue_left"], label="Blue_left", color="blue")
plt.plot(dfBlue.index, dfBlue["Alpha_left"], label="Alpha_left", color="orange")

# Add labels, title, and legend
plt.xlabel("Index")
plt.ylabel("Values")
plt.title("Blue Left")
plt.legend(loc="upper left")  # Legend inside the plot
plt.ylim(0, 500)

# Display the plot
plt.show()

# ------------------------BLUE RIGHT---------------------------#

# Initialize the plot
plt.figure(figsize=(10, 6))

# Plot different columns for Yellow category
plt.plot(dfBlue.index, dfBlue["Red_right"], label="Red_Right", color="red")
plt.plot(dfBlue.index, dfBlue["Green_right"], label="Green_Right", color="green")
plt.plot(dfBlue.index, dfBlue["Blue_right"], label="Blue_Right", color="blue")
plt.plot(dfBlue.index, dfBlue["Alpha_right"], label="Alpha_Right", color="orange")

# Add labels, title, and legend
plt.xlabel("Index")
plt.ylabel("Values")
plt.title("Blue Right")
plt.legend(loc="upper left")  # Legend inside the plot
plt.ylim(0, 500)
# Display the plot
plt.show()
