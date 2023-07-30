import matplotlib.pyplot as plt
import numpy as np

# Sample data points
x = np.random.rand(50)
y = np.random.rand(50)
z = np.random.randint(-10, 11, size=50)  # z values range from -10 to 10

# Create a 3D Axes object
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the 3D scatter plot with colormap 'plasma'
scatter = ax.scatter(x, y, z, c=z, cmap='plasma', edgecolors='k')

# Set labels for the axes
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Create a separate ScalarMappable object for the colorbar
scalarmappaple = plt.cm.ScalarMappable(cmap='plasma')
scalarmappaple.set_array(range(0,10))
cbar = plt.colorbar(scalarmappaple, ax=ax, label='Z Value')

plt.title('3D Scatter Plot with Colorbar and Fixed Grid Size')
plt.show()