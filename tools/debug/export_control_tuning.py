"""Export every completed or interrupted tuning trace to CSV and reproducible plots.

职责：从只读试验JSONL导出CSV、索引和预览图；不筛除失败数据。
线程：离线调用，无设备访问。
输入：试验目录；输出：独立derived子目录，源文件保持原样。
状态所有权：仅拥有衍生文件；禁止依赖：设备IO与参数修改。
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))


def flatten(value, prefix=''):
    if isinstance(value, dict):
        return {k:v for name,item in value.items() for k,v in flatten(item,f'{prefix}.{name}' if prefix else name).items()}
    if isinstance(value, (tuple,list)):
        if any(isinstance(item,(str,dict)) for item in value):
            return {prefix:json.dumps(value,ensure_ascii=False)}
        return {k:v for i,item in enumerate(value) for k,v in flatten(item,f'{prefix}.{i:02d}').items()}
    return {prefix:value}


def export(root):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['svg.fonttype']='none'
    plt.rcParams['pdf.fonttype']=42
    index=[]
    for source in sorted(root.rglob('*.jsonl')):
        if 'derived' in source.parts: continue
        if source.stem=='packets':
            with source.open('rb') as stream:
                checksum=hashlib.file_digest(stream,'sha256').hexdigest()
            with source.open(encoding='utf-8') as stream:
                count=sum(1 for _ in stream)
            index.append(dict(source=str(source.relative_to(root)),rows=count,sha256=checksum,
                              format='validated packet payload JSONL; retained without CSV expansion'))
            continue
        rows=[]; malformed=0
        for line in source.read_text(encoding='utf-8').splitlines():
            try: rows.append(json.loads(line))
            except json.JSONDecodeError: malformed+=1
        dest=source.parent/'derived'; dest.mkdir(exist_ok=True)
        flat=[flatten(r) for r in rows]
        columns=sorted({key for row in flat for key in row})
        with (dest/(source.stem+'.csv')).open('w',newline='',encoding='utf-8-sig') as stream:
            writer=csv.DictWriter(stream,fieldnames=columns); writer.writeheader(); writer.writerows(flat)
        entry=dict(source=str(source.relative_to(root)),rows=len(rows),malformed_lines=malformed,
                   sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                   status='see events; samples alone do not imply success')
        trace=[r for r in rows if all(k in r for k in ('t','q','target'))]
        if trace:
            from desktop.core.control_tuning import square_metrics, sine_metrics, metrics, reversal_metrics
            metadata_path=source.with_suffix('.metadata.json')
            metadata=json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
            results_path=source.parent/'results.json'
            if not metadata and results_path.exists() and source.stem.split('-')[0].isdigit():
                results=json.loads(results_path.read_text())
                ordinal=int(source.stem.split('-')[0])
                if ordinal<len(results): metadata=results[ordinal]
            trial=metadata.get('trial',{})
            analysis={'tracking':metrics(trace,discard_s=trial.get('discard',0)),
                      'analysis_source_sha256':hashlib.sha256((Path(__file__).resolve().parents[2]/'desktop/core/control_tuning.py').read_bytes()).hexdigest()}
            if trial.get('kind')=='square': analysis['edges']=square_metrics(trace)
            if trial.get('kind')=='sine': analysis['sine']=sine_metrics(trace,trial['frequency'],trial.get('discard',0))
            if trial.get('kind') in ('sine','triangle'):
                analysis['reversals']=reversal_metrics(trace,trial['frequency'])
            (dest/(source.stem+'.analysis.json')).write_text(json.dumps(analysis,indent=2))
            entry['trial']=metadata
            fig,axes=plt.subplots(2,1,sharex=True,figsize=(10,6))
            t=[r['t'] for r in trace]
            axes[0].plot(t,[r['target'] for r in trace],label='Applied target')
            axes[0].plot(t,[r['q'] for r in trace],label='Encoder q_fast')
            axes[0].set_ylabel('Angle (deg)'); axes[0].legend()
            axes[1].plot(t,[r['target']-r['q'] for r in trace])
            axes[1].set_ylabel('Error (deg)'); axes[1].set_xlabel('P4 elapsed time (s)')
            for ax in axes: ax.grid(alpha=.3)
            label=metadata.get('parameter','candidate')
            unit={'rate':' counts/s','initial':' counts','maximum':' counts','growth':' counts/s','ki':' /s','fast':' ms'}.get(label,'')
            joint=f"J{metadata['joint']:02d} | " if 'joint' in metadata else ''
            fig.suptitle(joint+source.stem+f" | {label}={metadata.get('point','unknown')}{unit}\nCompletion status: events.jsonl")
            fig.tight_layout()
            for suffix in ('png','svg','pdf'): fig.savefig(dest/(source.stem+'.'+suffix),dpi=300)
            plt.close(fig)
        index.append(entry)
    (root/'experiment-index.json').write_text(json.dumps(index,indent=2),encoding='utf-8')
    (root/'DATA-DICTIONARY.md').write_text(
        'Sources are immutable. CSV uses UTF-8 BOM; dotted suffixes identify array indices.\n'
        't: P4 elapsed seconds; host_elapsed: host monotonic elapsed seconds; p4_ms: P4 milliseconds.\n'
        'q/target: degrees. actual_abs/commands/P/I/reversal/recovery: servo counts.\n'
        'load/temperature_c/collision_armed/collision are latest asynchronous status, not same-cycle control fields; telemetry_host_rx is their available servo telemetry host timestamp.\n'
        'Empty cells are unavailable, not zero. Validity masks must be respected.\n'
        'Packet payload_hex, where present, is a validated protocol payload, not an unfiltered USB byte capture.\n'
        'Failed/interrupted attempts remain in events and the index; figures do not establish acceptance.\n',encoding='utf-8')
    return index


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path)
    export(parser.parse_args().root.resolve())
