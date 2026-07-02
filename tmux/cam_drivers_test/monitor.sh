#!/bin/bash

PROCESS_NAME=$1
DURATION=${2:-60}
SAVE_LOGS=${3:-"yes"}  # Pass "no" as 3rd argument to skip saving logs

echo "Waiting for process '$PROCESS_NAME' to start..."

get_pid() {
    ps -eo pid,cmd | grep "$PROCESS_NAME" | grep -v "grep" | grep -v "monitor.sh" | awk '{print $1}' | head -n 1
}

while [ -z "$(get_pid)" ]; do
    sleep 1
    echo -n "."
done

echo ""
echo "Process found!"

PID=$(get_pid)
echo "Monitoring PID: $PID"
echo "Duration: $DURATION seconds"
echo "Save logs: $SAVE_LOGS"
echo ""

# Setup output file only if saving is enabled
if [ "$SAVE_LOGS" = "yes" ]; then
    LOG_DIR="$(pwd)/logs"
    mkdir -p "$LOG_DIR"
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    OUTPUT_FILE="$LOG_DIR/${PROCESS_NAME}_${TIMESTAMP}_full_log.csv"
    echo "timestamp,cpu_percent,mem_percent,ram_mb,gpu_util,gpu_mem_mb" > "$OUTPUT_FILE"
    echo "Saving to: $OUTPUT_FILE"
fi

for i in $(seq 1 $DURATION); do
    if ! ps -p $PID > /dev/null 2>&1; then
        echo "WARNING: Process $PID terminated at sample $i"
        break
    fi

    DATA=$(ps -p $PID -o %cpu,%mem,rss --no-headers)

    if [ ! -z "$DATA" ]; then
        CPU=$(echo $DATA | awk '{print $1}')
        MEM=$(echo $DATA | awk '{print $2}')
        RAM_KB=$(echo $DATA | awk '{print $3}')
        RAM_MB=$(awk "BEGIN {printf \"%.2f\", $RAM_KB/1024}")

        GPU_DATA=$(nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader,nounits 2>/dev/null || echo "0, 0")
        GPU_UTIL=$(echo $GPU_DATA | awk -F',' '{print $1}')
        GPU_MEM=$(echo $GPU_DATA | awk -F',' '{print $2}')

        if [ "$SAVE_LOGS" = "yes" ]; then
            echo "$(date +%s),$CPU,$MEM,$RAM_MB,$GPU_UTIL,$GPU_MEM" >> "$OUTPUT_FILE"
        fi

        if [ $((i % 10)) -eq 0 ]; then
            echo "[$i/$DURATION] CPU: $CPU% | RAM: $RAM_MB MB | GPU: $GPU_UTIL% | VRAM: $GPU_MEM MB"
        fi
    fi

    sleep 1
done

echo ""
echo "=== RESULTS ==="

if [ "$SAVE_LOGS" = "yes" ]; then
    echo "Data saved to: $OUTPUT_FILE"
    awk -F',' 'NR>1 {
        cpu+=$2; mem+=$3; ram+=$4; gpu+=$5; vram+=$6;
        if ($2 > max_cpu) max_cpu = $2;
        if ($4 > max_ram) max_ram = $4;
        if ($5 > max_gpu) max_gpu = $5;
        if ($6 > max_vram) max_vram = $6;
        count++
    }
    END {
        if (count > 0) {
            printf "Average CPU: %.1f %%\n", cpu/count;
            printf "Peak CPU: %.1f %%\n", max_cpu;
            printf "Average RAM: %.2f GB\n", ram/count/1024;
            printf "Peak RAM: %.2f GB\n", max_ram/1024;
            printf "Average GPU: %.1f %%\n", gpu/count;
            printf "Peak GPU: %.1f %%\n", max_gpu;
            printf "Average VRAM: %.2f GB\n", vram/count/1024;
            printf "Peak VRAM: %.2f GB\n", max_vram/1024;
            printf "Samples: %d\n", count;
        }
    }' "$OUTPUT_FILE"
else
    echo "Log saving was disabled — no file written."
fi

echo ""
echo "Thread count: $(ps -p $PID -L 2>/dev/null | wc -l)"
