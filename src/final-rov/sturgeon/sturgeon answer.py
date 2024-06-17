import csv
import matplotlib.pyplot as plt

def read_csv(filename):
    table = []
    with open(filename, 'r') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            table.append(row)
    return table

def find_max_period(csv_data):
    max_sum = 0
    max_receiver = None
    max_period_start = None

    for receiver_index in range(1, len(csv_data)):
        for day_start in range(1, len(csv_data[0]) - 4):
            period_sum = sum(int(csv_data[receiver_index][day]) for day in range(day_start, day_start + 5))
            if period_sum > max_sum:
                max_sum = period_sum
                max_receiver = receiver_index
                max_period_start = day_start

    return max_receiver, max_period_start, max_period_start + 4

def plot_table(csv_data, max_receiver, period_start, period_end, output_filename):
    days = [int(day) for day in csv_data[0][1:]]
    
    for i, receiver_data in enumerate(csv_data[1:], start=1):
        receiver_label = f"Receiver {i}"
        receiver_values = [int(val) for val in receiver_data[1:]]
        if i == max_receiver:
            plt.plot(days, receiver_values, label=receiver_label, color='red')
        else:
            plt.plot(days, receiver_values, label=receiver_label)

    plt.axvspan(period_start, period_end, color='yellow', alpha=0.3)
    plt.title("Sturgeon Numbers Over 15 Days")
    plt.xlabel("Day #")
    plt.ylabel("# of Sturgeon")
    plt.legend()
    plt.grid(True)
    plt.savefig(output_filename)    
    plt.show()

def detect(filename):
    csv_data = read_csv(filename)
    max_receiver, period_start, period_end = find_max_period(csv_data)
    print(f"The receiver with the maximum sum of 5 days is Receiver {max_receiver}, with the period from day {period_start} to day {period_end}.")
    plot_table(csv_data, max_receiver, period_start, period_end, f"{filename[:-4]}.png")

for i in range(10):
    detect(f"sturgeon_data {i+1}.csv")    
