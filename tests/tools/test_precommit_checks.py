"""Commit gate must refuse incomplete staging, missing tools and failing/skipped tests."""
import importlib.metadata
from pathlib import Path
import subprocess
import pytest
from tools import precommit_checks as gate


def test_partial_staging_is_rejected_without_modifying_index(tmp_path, monkeypatch):
    subprocess.run(['git', 'init', '-q', str(tmp_path)], check=True)
    source = tmp_path / 'control.py'
    source.write_text('value = 1\n')
    subprocess.run(['git', '-C', str(tmp_path), 'add', 'control.py'], check=True)
    source.write_text('value = 2\n')
    monkeypatch.setattr(gate, 'ROOT', tmp_path)
    with pytest.raises(RuntimeError, match='Stage the complete'):
        gate.check_staging()
    assert subprocess.check_output(['git', '-C', str(tmp_path), 'show', ':control.py']) == b'value = 1\n'


@pytest.mark.parametrize('attributes', ['tests="1" skipped="1"', 'tests="1" failures="1"', 'tests="0"'])
def test_incomplete_reports_are_rejected(tmp_path, attributes):
    report = tmp_path / 'report.xml'
    report.write_text(f'<testsuites><testsuite {attributes}/></testsuites>')
    with pytest.raises(RuntimeError, match='Commit refused'):
        gate.check_test_report(report)


def test_missing_compiler_is_rejected(monkeypatch):
    monkeypatch.setattr(gate.shutil, 'which', lambda *args, **kwargs: None)
    monkeypatch.setenv('ProgramFiles(x86)', 'X:/missing-build-tools')
    with pytest.raises(RuntimeError, match='compiler is required'):
        gate.compiler_environment()


@pytest.mark.parametrize('failure', ['generated', 'pytest'])
def test_failed_required_command_refuses_commit(monkeypatch, failure):
    monkeypatch.setattr(gate, 'check_dependencies', lambda: None)
    monkeypatch.setattr(gate, 'compiler_environment', lambda: {})
    monkeypatch.setattr(gate.shutil, 'which', lambda *_args: 'powershell')
    def fail(args, **kwargs):
        if (failure == 'generated' and any('generate_robot_model' in str(a) for a in args)) or (failure == 'pytest' and 'pytest' in args):
            raise subprocess.CalledProcessError(1, args)
    monkeypatch.setattr(gate, 'command', fail)
    assert gate.main([]) == 1


def test_missing_python_dependency_refuses_commit(monkeypatch):
    def missing(): raise importlib.metadata.PackageNotFoundError('pytest')
    monkeypatch.setattr(gate, 'check_dependencies', missing)
    assert gate.main([]) == 1
