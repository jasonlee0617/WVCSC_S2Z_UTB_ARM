#!/usr/bin/env python3
"""Extract /vision/visual_servo_debug JSON strings from a rosbag into JSONL + summary."""
import argparse
import json
from pathlib import Path


def _db3_files(bag_dir):
    root = Path(bag_dir)
    if not root.is_dir():
        raise FileNotFoundError(f'bag directory not found: {bag_dir}')
    db3 = sorted(root.rglob('*.db3'))
    if not db3:
        raise FileNotFoundError(f'no .db3 files found under {bag_dir}')
    return db3


def _messages(db3_paths, topic):
    """Yield (timestamp_ns, topic_name, data_bytes) tuples using sqlite3."""
    import sqlite3

    for db3 in db3_paths:
        conn = sqlite3.connect(f'file:{db3}?mode=ro', uri=True)
        conn.row_factory = sqlite3.Row
        # Find the topic id
        topic_rows = conn.execute(
            'SELECT id FROM topics WHERE name = ?', (topic,)).fetchall()
        if not topic_rows:
            conn.close()
            continue
        topic_id = topic_rows[0]['id']
        rows = conn.execute(
            'SELECT timestamp, data FROM messages WHERE topic_id = ? '
            'ORDER BY timestamp', (topic_id,))
        for row in rows:
            yield row['timestamp'], topic, row['data']
        conn.close()


def _extract(bag_dir, topic, out_jsonl, out_summary):
    db3 = _db3_files(bag_dir)
    events = []
    error_px_samples = []

    with open(out_jsonl, 'w', encoding='utf-8') as jf:
        for _ts_ns, _topic, data in _messages(db3, topic):
            try:
                payload = data.decode('utf-8')
                msg = json.loads(payload)
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            jf.write(payload + '\n')

            event = msg.get('event', '')
            if event and event != 'control':
                events.append(msg)
            if msg.get('error_u_px') is not None and msg.get('error_v_px') is not None:
                error_px_samples.append(
                    (msg.get('elapsed_sec', 0.0),
                     msg['error_u_px'], msg['error_v_px']))

    # ── summary ──
    lines = []
    lines.append(f'Total debug messages: {len(error_px_samples) + len(events)}')
    lines.append(f'  control samples: {len(error_px_samples)}')
    lines.append(f'  event samples:   {len(events)}')
    lines.append('')

    alignments = [e for e in events if e.get('event') == 'aligned']
    timeouts = [e for e in events if e.get('event') == 'timeout']
    stalled = [e for e in events if e.get('event') == 'servo_singularity']
    safety = [e for e in events if e.get('event') == 'servo_safety_stop']

    lines.append(f'Alignments:  {len(alignments)} succeeded')
    if alignments:
        for a in alignments:
            lines.append(
                f'  target={a.get("target_id","?")} '
                f'elapsed={a.get("elapsed_sec",0):.2f}s '
                f'error=({a.get("error_u_px",0):.1f},{a.get("error_v_px",0):.1f})px')

    lines.append(f'Timeouts:    {len(timeouts)}')
    for t in timeouts:
        lines.append(
            f'  target={t.get("target_id","?")} '
            f'elapsed={t.get("elapsed_sec",0):.2f}s '
            f'error=({t.get("error_u_px",0):.1f},{t.get("error_v_px",0):.1f})px')

    lines.append(f'Singularity: {len(stalled)}')
    lines.append(f'Safety stop: {len(safety)}')
    lines.append('')

    # Statistics from control samples
    if error_px_samples:
        import math
        norms = [math.hypot(eu, ev) for _, eu, ev in error_px_samples]
        norms_sorted = sorted(norms)
        lines.append('Control error statistics (px, Euclidean norm):')
        lines.append(f'  count:  {len(norms)}')
        lines.append(f'  min:    {min(norms):.1f}')
        lines.append(f'  median: {norms_sorted[len(norms_sorted)//2]:.1f}')
        lines.append(f'  max:    {max(norms):.1f}')
        lines.append(f'  mean:   {sum(norms)/len(norms):.1f}')

    with open(out_summary, 'w', encoding='utf-8') as sf:
        sf.write('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--topic', default='/vision/visual_servo_debug')
    parser.add_argument('--out', required=True)
    parser.add_argument('--summary', required=True)
    args = parser.parse_args()
    _extract(args.bag, args.topic, args.out, args.summary)
    print(f'Wrote {args.out}')
    print(f'Wrote {args.summary}')


if __name__ == '__main__':
    main()
