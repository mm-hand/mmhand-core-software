"""Contract checks for the English browser interface."""
import ast
from pathlib import Path


def test_browser_labels_and_templates_have_no_chinese_text():
    source = Path(__file__).resolve().parents[3] / 'desktop/ui/browser_view.py'
    sources = [source, *source.with_name('web').glob('*.py')]
    trees = [ast.parse(path.read_text(encoding='utf-8')) for path in sources]
    docstrings = {id(node.value) for tree in trees for parent in ast.walk(tree)
                  if isinstance(parent, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
                  for node in parent.body[:1] if isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant)}
    failures = [node.value for tree in trees for node in ast.walk(tree)
                if isinstance(node, ast.Constant) and isinstance(node.value, str) and id(node) not in docstrings
                and any('\u4e00' <= char <= '\u9fff' for char in node.value)]
    assert not failures
