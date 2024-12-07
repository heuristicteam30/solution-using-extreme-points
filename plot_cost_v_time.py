import matplotlib.pyplot as plt

# Example data
time_array = [0, 1, 2, 3, 4, 5]  # Time in seconds
cost_array = [100, 80, 60, 45, 30, 20]  # Left-over cost

# Plot
plt.plot(time_array, cost_array, marker='s', linestyle='--', color='b')
plt.xlabel('Time (seconds)')
plt.ylabel('Left-over Cost')
plt.title('Left-over Cost vs Time of Execution')
plt.grid()

# Save the plot
plt.savefig('leftover_cost_vs_time.png')
