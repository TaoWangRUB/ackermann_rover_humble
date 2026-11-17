#!/usr/bin/env bash
# List the boots in the journal in the right order, with their real start and
# end times, or print one boot's ID.
#
#   journal_boots.sh          # table: offset (0 = this boot), boot ID, start, end
#   journal_boots.sh -1       # the previous boot's ID, e.g.
#   journalctl -b "$(scripts/rover_units/journal_boots.sh -1)" -u rover-t265
#
# Why not `journalctl --list-boots` / `journalctl -b -1`: on the Xavier the
# kernel sets the clock from the Tegra RTC (rtc1), which restarts at 1970 on
# every boot. systemd then moves it to its own build date (2024-06-17) until
# timesyncd restores the real time a few seconds later, so about 1,000 entries
# of each boot carry 2024 timestamps. journalctl orders boots by their first
# entry, so the current boot seems to start in 2024 and hides the earlier ones.
# Here each boot is placed by its LAST entry, and its start is that entry's
# wall-clock time minus its time since boot.
set -euo pipefail

want="${1:-}"
if [[ -n "${want}" && ! "${want}" =~ ^(0|-[0-9]+)$ ]]; then
    echo "usage: journal_boots.sh [0|-N]" >&2
    exit 2
fi

rows=()
while read -r boot; do
    [[ -n "${boot}" ]] || continue
    # Last entry of that boot: wall-clock and monotonic time in microseconds.
    read -r real mono < <(journalctl "_BOOT_ID=${boot}" -n 1 -o export --no-pager 2>/dev/null \
        | awk -F= '/^__REALTIME_TIMESTAMP=/{r=$2} /^__MONOTONIC_TIMESTAMP=/{m=$2} END{print r, m}')
    [[ -n "${real}" && -n "${mono}" ]] || continue
    rows+=("${real} $(( real - mono )) ${boot}")
done < <(journalctl -F _BOOT_ID --no-pager 2>/dev/null)

(( ${#rows[@]} > 0 )) || { echo "no boots found in the journal" >&2; exit 1; }
mapfile -t sorted < <(printf '%s\n' "${rows[@]}" | sort -n)

n=${#sorted[@]}
for i in "${!sorted[@]}"; do
    read -r end start boot <<< "${sorted[i]}"
    offset=$(( i - n + 1 ))
    if [[ -n "${want}" ]]; then
        (( offset == want )) && { echo "${boot}"; exit 0; }
        continue
    fi
    printf '%4d  %s  %s  ->  %s\n' "${offset}" "${boot}" \
        "$(date -d "@$(( start / 1000000 ))" '+%F %T')" "$(date -d "@$(( end / 1000000 ))" '+%F %T')"
done
if [[ -n "${want}" ]]; then
    echo "no boot at offset ${want} (the journal holds ${n})" >&2
    exit 1
fi
