import psutil
import csv
import time

def monitor_system_usage(interval=1, output_file='system_usage.csv'):
    with open(output_file, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Time', 'CPU Usage (%)', 'Memory Usage (%)'])

        try:
            while True:
                cpu_usage = psutil.cpu_percent(interval=interval)
                memory_usage = psutil.virtual_memory().percent
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

                writer.writerow([timestamp, cpu_usage, memory_usage])

        except KeyboardInterrupt:
            print("Monitoring interrupted. Data saved to 'system_usage.csv'.")

if __name__ == "__main__":
    monitor_system_usage(interval=1, output_file='system_usage.csv')
