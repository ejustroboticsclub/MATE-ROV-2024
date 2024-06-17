import random
import matplotlib.pyplot as plt
import csv

def generate_table():
    table = [[random.randint(0, 15) for _ in range(15)] for _ in range(3)]
    return table

def add_period_with_max_values(table):
    receiver_index = random.randint(0, 2)
    start_day = random.randint(0, 10)  # Start day to ensure 5 days period
    for day in range(start_day, start_day + 5):
        table[receiver_index][day] = random.randint(15, 25)  # Assign higher values for this period
    return table

def print_table(table):
    print("Day ", end="")
    for i in range(1, 16):
        print(i, end=" ")
    print()
    for i, row in enumerate(table):
        print(f"Receiver {i+1} ", end="")
        for val in row:
            print(val, end=" ")
        print()

def save_csv(table, name):
    with open(f'sturgeon_data {name}.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Day"] + list(range(1, 16)))
        for i, row in enumerate(table):
            writer.writerow([f"Receiver {i+1}"] + row)

for i in range(10):
    print(f"Table Variation {i+1}:")
    table = generate_table()
    table = add_period_with_max_values(table)
    print_table(table)
    save_csv(table, i+1)
    print()
