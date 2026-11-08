#!/usr/bin/env python3
"""Ch 18 capstone: per-stage times, boxes per minute and retries from an executive log.

    ros2 launch cell_bringup cell.launch.py 2>&1 | tee run.log
    python3 tools/job_stats.py run.log
"""
import re, sys, collections

STAGE = re.compile(r'\[(\d+\.\d+)\].*\[executive[^\]]*\].*?\[(\d+)\] (\w+)')      # [ts] [executive]: [moved] STATE
RECOVER = re.compile(r'recover \((\d)/3\)')


def main(path):
    times = collections.defaultdict(list)
    last = None
    first = end = None
    moved = 0
    retries = 0
    for line in open(path):
        m = STAGE.search(line)
        if m:
            t, moved, state = float(m.group(1)), int(m.group(2)), m.group(3)
            first = first or t; end = t
            if last:
                times[last[1]].append(t - last[0])
            last = (t, state)
        if RECOVER.search(line):
            retries += 1
    if not first:
        print('no executive states found (launch with output=screen and keep the timestamps)'); return
    total = end - first
    print(f'boxes moved: {moved}   wall time: {total:.1f} s   boxes/min: {60 * moved / max(total, 1e-9):.1f}   recoveries: {retries}')
    for state, ts in sorted(times.items(), key=lambda kv: -sum(kv[1])):
        print(f'  {state:<10} {len(ts):>3} x  mean {sum(ts) / len(ts):6.2f} s  total {sum(ts):7.1f} s')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'run.log')
