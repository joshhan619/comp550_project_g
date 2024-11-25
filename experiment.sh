#!/bin/bash

# Array of values for n
n_values=(1000 10000 100000 1000000)
m=10
iterations=20

for n in "${n_values[@]}"; do
    for ((i=1; i<=iterations; i++)); do
        # Run the C++ program with the current n and m
        ./ProjectG "$n" "$m"
        
        # Check if the output file exists
        output_file="output${n}.txt"
    done
done
