"""Read-only commit gate shared by developers and the Git pre-commit hook."""
from __future__ import annotations
import argparse
import importlib.metadata
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SOURCE_SUFFIXES = {'.py', '.cpp', '.h', '.ino', '.json', '.sh', '.yaml', '.yml', '.toml', '.ini', '.txt'}


def command(args, *, env=None):
    subprocess.run(args, cwd=ROOT, env=env, check=True)


def check_dependencies():
    for package, version in [('pytest', '8.4.2'), ('nicegui', '3.16.0'), ('numpy', None), ('pyserial', None)]:
        installed = importlib.metadata.version(package)
        if version and installed != version:
            raise RuntimeError(f'{package} must be {version}; found {installed}. Install requirements-test.txt.')


def compiler_environment():
    env = os.environ.copy()
    if any(shutil.which(name) for name in ('g++', 'clang++', 'cl')):
        return env
    if os.name == 'nt':
        # VS installs its compiler outside PATH. Read the official developer environment.
        locator = Path(os.environ.get('ProgramFiles(x86)', 'C:/Program Files (x86)')) / 'Microsoft Visual Studio/Installer/vswhere.exe'
        if locator.is_file():
            installation = subprocess.check_output([str(locator), '-latest', '-products', '*', '-requires', 'Microsoft.VisualStudio.Component.VC.Tools.x86.x64', '-property', 'installationPath'], text=True).strip()
            batch = Path(installation) / 'VC/Auxiliary/Build/vcvars64.bat'
            if installation and batch.is_file():
                output = subprocess.check_output(f'call "{batch}" >nul && set', shell=True, text=True, errors='replace')
                for line in output.splitlines():
                    key, separator, value = line.partition('=')
                    if separator and key and not key.startswith('='):
                        env[key.upper()] = value
                if shutil.which('cl', path=env.get('PATH')):
                    return env
    raise RuntimeError('A host C++ compiler is required (MSVC Build Tools, g++ or clang++). Native tests may not be skipped.')


def check_staging():
    def names(*args):
        return subprocess.check_output(['git', *args, '-z'], cwd=ROOT).decode('utf-8').split('\0')
    unstaged = names('diff', '--name-only') + names('ls-files', '--others', '--exclude-standard')
    problems = [name for name in unstaged if name and (Path(name).suffix.lower() in SOURCE_SUFFIXES or name.startswith('.githooks/'))]
    if problems:
        raise RuntimeError('Stage the complete source/test/config changes before commit: ' + ', '.join(problems))
    command(['git', 'diff', '--cached', '--check'])


def check_test_report(path):
    root = ET.parse(path).getroot()
    suites = [root] if root.tag == 'testsuite' else list(root.iter('testsuite'))
    tests = sum(int(s.get('tests', 0)) for s in suites)
    skipped = sum(int(s.get('skipped', 0)) for s in suites)
    failed = sum(int(s.get('failures', 0)) + int(s.get('errors', 0)) for s in suites)
    if not tests or skipped or failed:
        raise RuntimeError(f'Mandatory tests: {tests} collected, {failed} failed, {skipped} skipped. Commit refused.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--staged', action='store_true', help='also require all source/test/config changes to be staged')
    args = parser.parse_args(argv)
    try:
        check_dependencies()
        env = compiler_environment()
        if args.staged:
            check_staging()
        for script in ('generate_upper_protocol.py', 'generate_robot_model.py', 'generate_p4_safety_catalog.py',
                       'generate_control_parameters.py'):
            command([sys.executable, str(ROOT / 'tools' / script), '--check'], env=env)
        command([sys.executable, str(ROOT / 'tools/check_desktop_architecture.py')], env=env)
        command([sys.executable, str(ROOT / 'tools/p4_architecture/check_p4_architecture.py')], env=env)
        # Git hooks export repository-local paths; isolated test repos must not inherit them.
        for key in ('GIT_DIR', 'GIT_WORK_TREE', 'GIT_INDEX_FILE', 'GIT_COMMON_DIR', 'GIT_PREFIX'):
            env.pop(key, None)
        env['PYTEST_DISABLE_PLUGIN_AUTOLOAD'] = '1'
        env['PYTHONIOENCODING'] = 'utf-8'
        with tempfile.TemporaryDirectory(prefix='mmhand-checks-') as directory:
            report = Path(directory) / 'results.xml'
            env['MMHAND_DATA_DIR'] = str(Path(directory) / 'data')
            command([sys.executable, '-m', 'pytest', '-q', '--basetemp', str(Path(directory) / 'pytest'), '--junitxml', str(report)], env=env)
            check_test_report(report)
    except (RuntimeError, subprocess.CalledProcessError, importlib.metadata.PackageNotFoundError) as exc:
        print(f'Commit gate FAILED: {exc}', file=sys.stderr)
        return 1
    print('Commit gate PASSED (all software tests, no mandatory skips).')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
