#!/bin/sh
set -eu

OUT_DIR="${1:-/tmp/core_irq_probe}"
DURATION="${DURATION:-60}"
INTERVAL="${INTERVAL:-1}"
IRQ_CORE1_MASK="${IRQ_CORE1_MASK:-2}"

mkdir -p "$OUT_DIR"

now_ms()
{
    awk '{printf "%d\n", $1 * 1000}' /proc/uptime
}

snapshot_once()
{
    tag="$1"
    dir="$OUT_DIR/$tag"
    mkdir -p "$dir"

    uname -a > "$dir/uname.txt"
    cat /proc/cmdline > "$dir/cmdline.txt"
    cat /proc/cpuinfo > "$dir/cpuinfo.txt"
    cat /proc/interrupts > "$dir/interrupts.txt"
    cat /proc/softirqs > "$dir/softirqs.txt" 2>/dev/null || true
    cat /proc/stat > "$dir/stat.txt"
    cat /proc/loadavg > "$dir/loadavg.txt"
    ps > "$dir/ps.txt"

    {
        for irq_dir in /proc/irq/[0-9]*; do
            [ -d "$irq_dir" ] || continue
            irq="${irq_dir##*/}"
            printf 'irq=%s\n' "$irq"
            if [ -r "$irq_dir/actions" ]; then
                printf 'actions='
                cat "$irq_dir/actions"
            fi
            if [ -r "$irq_dir/smp_affinity" ]; then
                printf 'smp_affinity='
                cat "$irq_dir/smp_affinity"
            fi
            if [ -r "$irq_dir/smp_affinity_list" ]; then
                printf 'smp_affinity_list='
                cat "$irq_dir/smp_affinity_list"
            fi
            echo ""
        done
    } > "$dir/irq_affinity.txt"

    pid="$(pidof cam_system 2>/dev/null || true)"
    if [ -n "$pid" ]; then
        dump_threads "$pid" > "$dir/cam_threads.txt"
    fi
}

dump_threads()
{
    pid="$1"
    for task in /proc/"$pid"/task/[0-9]*; do
        [ -d "$task" ] || continue
        tid="${task##*/}"
        printf 'tid=%s\n' "$tid"
        if [ -r "$task/status" ]; then
            awk '/^(Name|State|Cpus_allowed|Cpus_allowed_list|voluntary_ctxt_switches|nonvoluntary_ctxt_switches):/ {print}' "$task/status"
        fi
        if [ -r "$task/stat" ]; then
            awk '{print "stat_utime="$14, "stat_stime="$15, "last_cpu="$39}' "$task/stat"
        fi
        echo ""
    done
}

save_irq_affinity()
{
    save_file="$OUT_DIR/irq_affinity.restore"
    : > "$save_file"
    for irq_dir in /proc/irq/[0-9]*; do
        [ -r "$irq_dir/smp_affinity" ] || continue
        irq="${irq_dir##*/}"
        value="$(cat "$irq_dir/smp_affinity")"
        printf '%s %s\n' "$irq" "$value" >> "$save_file"
    done
    echo "$save_file"
}

apply_irq_core1()
{
    save_irq_affinity >/dev/null
    log="$OUT_DIR/apply_irq_core1.log"
    : > "$log"
    for irq_dir in /proc/irq/[0-9]*; do
        [ -w "$irq_dir/smp_affinity" ] || continue
        irq="${irq_dir##*/}"
        actions=""
        [ -r "$irq_dir/actions" ] && actions="$(cat "$irq_dir/actions")"
        if echo "$IRQ_CORE1_MASK" > "$irq_dir/smp_affinity" 2>/dev/null; then
            printf 'OK irq=%s actions=%s mask=%s\n' "$irq" "$actions" "$IRQ_CORE1_MASK" >> "$log"
        else
            printf 'FAIL irq=%s actions=%s mask=%s\n' "$irq" "$actions" "$IRQ_CORE1_MASK" >> "$log"
        fi
    done
}

restore_irq_affinity()
{
    save_file="$OUT_DIR/irq_affinity.restore"
    [ -f "$save_file" ] || {
        echo "No restore file: $save_file" >&2
        return 1
    }

    while read -r irq value; do
        [ -n "$irq" ] || continue
        if [ -w "/proc/irq/$irq/smp_affinity" ]; then
            echo "$value" > "/proc/irq/$irq/smp_affinity" 2>/dev/null || true
        fi
    done < "$save_file"
}

collect_loop()
{
    if [ "$INTERVAL" -lt 1 ]; then
        echo "INTERVAL must be >= 1" >&2
        exit 1
    fi

    samples=$((DURATION / INTERVAL))
    [ "$samples" -lt 1 ] && samples=1

    snapshot_once before
    metrics="$OUT_DIR/samples.tsv"
    printf 't_ms\tloadavg\tcpu_line\tsoftirq_line\tcam_pid\tcam_threads\n' > "$metrics"

    i=0
    while [ "$i" -lt "$samples" ]; do
        t="$(now_ms)"
        load="$(cat /proc/loadavg)"
        cpu="$(awk '/^cpu / {print; exit}' /proc/stat)"
        softirq="$(awk '$1 ~ /^(NET_RX:|TIMER:|SCHED:)$/ {printf "%s ", $0}' /proc/softirqs 2>/dev/null || true)"
        pid="$(pidof cam_system 2>/dev/null || true)"
        thread_count="0"
        if [ -n "$pid" ] && [ -d "/proc/$pid/task" ]; then
            for task in /proc/"$pid"/task/[0-9]*; do
                [ -d "$task" ] || continue
                thread_count=$((thread_count + 1))
            done
            dump_threads "$pid" > "$OUT_DIR/cam_threads_$i.txt"
        fi
        printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$t" "$load" "$cpu" "$softirq" "${pid:-none}" "$thread_count" >> "$metrics"
        sleep "$INTERVAL"
        i=$((i + 1))
    done

    snapshot_once after
}

case "${MODE:-collect}" in
    collect)
        collect_loop
        ;;
    snapshot)
        snapshot_once snapshot
        ;;
    apply-core1)
        snapshot_once before_apply
        apply_irq_core1
        snapshot_once after_apply
        ;;
    restore)
        restore_irq_affinity
        snapshot_once after_restore
        ;;
    *)
        echo "Usage: MODE={collect|snapshot|apply-core1|restore} OUT_DIR DURATION=60 INTERVAL=1 $0" >&2
        exit 1
        ;;
esac
