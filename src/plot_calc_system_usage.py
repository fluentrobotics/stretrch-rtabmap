import pandas as pd
import matplotlib.pyplot as plt

def plot_and_calculate_avg_usage(csv_file):
    data = pd.read_csv(csv_file)
    data['Time'] = pd.to_datetime(data['Time'])

    time = data['Time'].values
    cpu_usage = data['CPU Usage (%)'].values
    memory_usage = data['Memory Usage (%)'].values

    avg_cpu_usage = cpu_usage.mean()
    avg_memory_usage = memory_usage.mean()

    plt.figure(figsize=(10, 6))
    plt.plot(time, cpu_usage, label='CPU Usage (%)', color='blue')
    plt.plot(time, memory_usage, label='Memory Usage (%)', color='red')
    plt.xlabel('Time')
    plt.ylabel('Usage (%)')
    plt.title('CPU and Memory Usage Over Time')
    plt.legend()
    plt.xticks(rotation=45)
    plt.tight_layout()

    plt.show()

    return avg_cpu_usage, avg_memory_usage

if __name__ == "__main__":
    csv_file = '/home/hello-robot/SR_stretch_ws/src/rtab_stretch/src/system_usage.csv'
    avg_cpu, avg_memory = plot_and_calculate_avg_usage(csv_file)
    print(f"Average CPU Usage: {avg_cpu}%")
    print(f"Average Memory Usage: {avg_memory}%")

