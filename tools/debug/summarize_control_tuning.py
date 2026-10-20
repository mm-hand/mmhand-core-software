"""汇总调参档案，不改写原始测量。

职责：建立跨试验状态索引和逐轨迹指标 CSV，保留失败和未完成状态。
线程：单一离线进程；输入：试验 events/results/metadata；输出：同目录汇总。
状态所有权：原始试验目录只读；禁止依赖：串口、GUI、设备控制。
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def summarize(root: Path):
    attempts, trials = [], []
    for folder in sorted(root.iterdir()):
        events_path = folder / 'events.jsonl'
        if not folder.is_dir() or not events_path.exists():
            continue
        events, malformed = [], 0
        for line in events_path.read_text(encoding='utf-8').splitlines():
            try:
                events.append(json.loads(line))
            except ValueError:
                malformed += 1
        endings = [e for e in events if e.get('kind') in ('COMPLETED', 'STOPPED')]
        results = [e for e in events if e.get('kind') == 'TRIAL_RESULT']
        connected = next((e for e in events if e.get('kind') == 'CONNECTED'), {})
        fingerprint = connected.get('fingerprint', {})
        status = endings[-1]['kind'] if endings else 'UNRESOLVED'
        attempts.append(dict(attempt=folder.name, status=status,
                             completed_trials=len(results), malformed_event_lines=malformed,
                             error=endings[-1].get('error') if endings else None,
                             firmware_source_crc=fingerprint.get('firmware_source_crc')))
        for result in results:
            trial, metrics = result['trial'], result['metrics']
            row = dict(attempt=folder.name, attempt_status=status,
                       firmware_source_crc=fingerprint.get('firmware_source_crc'),
                       case=trial['case'], trial=trial['name'], kind=trial['kind'],
                       point=result.get('point'), repeat=result.get('repeat'))

            def flatten(prefix, value):
                if isinstance(value, dict):
                    for key, item in value.items():
                        flatten(prefix + '.' + key if prefix else key, item)
                elif not isinstance(value, list):
                    row[prefix] = value

            flatten('metrics', metrics)
            trials.append(row)
    (root / 'attempts.json').write_text(json.dumps(attempts, indent=2, ensure_ascii=False), encoding='utf-8')
    keys = list(dict.fromkeys(key for row in trials for key in row))
    with (root / 'all-trial-metrics.csv').open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(trials)
    (root / 'SUMMARY-NOTES.md').write_text(
        '# 调参档案索引\n\n'
        f'共 {len(attempts)} 个试验档案，{len(trials)} 条已完成轨迹。此数量不代表验收通过。\n\n'
        '`attempts.json` 包含中断原因；`all-trial-metrics.csv` 每行是一条已完成轨迹。'
        '中断轨迹没有完整指标，仍保留在对应目录的 JSONL、事件及 derived 文件中。'
        'UNRESOLVED 表示缺少终结事件，可能仍运行或仅为诊断档案，不能当作完成。\n\n'
        '不同 firmware_source_crc、标定、预加载及参数条件不能直接合并为同一对照。'
        '使用各目录 metadata、parameters 和 reproduction 核对条件；保留原始数据和在线指标，'
        '最新离线指标见 derived/analysis.json。在线指标和后续修正后的离线指标可能不同。\n\n'
        '精度为磁编坐标精度；配对偏差不是张力；原始温度含孤立尖峰，尚未完成温度测量核验。'
        '失败的换向补偿候选不能作为推荐参数；尚未完成的 Case 不构成已验收结果。\n', encoding='utf-8')
    return dict(attempts=len(attempts), completed_trajectories=len(trials))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    print(json.dumps(summarize(parser.parse_args().root)))
