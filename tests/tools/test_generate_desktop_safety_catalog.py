"""Desktop 安全目录生成器的回归测试。"""

from __future__ import annotations

import ast
import textwrap

from tools import generate_desktop_safety_catalog as catalog


def test_text_templates_fstrings_and_plain_strings():
    node = ast.parse("f'M{m:02d} is offline'").body[0].value
    assert catalog._text(node) == "M{…} is offline"
    assert catalog._text(ast.Constant(value="plain")) == "plain"
    assert catalog._text(ast.Constant(value=3)) == ""


def test_rejections_cover_return_operation_and_reason(tmp_path):
    source = tmp_path / "sample.py"
    source.write_text(textwrap.dedent("""
        def a():
            return False, 'alpha'
            return True, 'ignored'
        def b():
            return OperationResult(False, None, 'beta')
        def c():
            reason = 'gamma failed'
            return False, reason
        def d():
            reason = 'internal_code'
            return False, reason
        def e():
            return OperationResult(False, None, IssueMessage('delta', 'X'))
    """), encoding="utf-8")
    assert {text for _, text in catalog._rejections(source)} == {"alpha", "beta", "gamma failed", "delta"}


def test_committed_catalog_is_current():
    assert catalog.main(["--check"]) == 0
